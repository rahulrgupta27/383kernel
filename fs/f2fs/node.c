/*
 * fs/f2fs/node.c
 *
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *             http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/fs.h>
#include <linux/f2fs_fs.h>
#include <linux/mpage.h>
#include <linux/backing-dev.h>
#include <linux/blkdev.h>
#include <linux/pagevec.h>
#include <linux/swap.h>

#include "f2fs.h"
#include "node.h"
#include "segment.h"

static struct kmem_cache *nat_entry_slab;
static struct kmem_cache *free_nid_slab;

static void clear_node_page_dirty(struct page *page)
{
	struct address_space *mapping = page->mapping;
	struct f2fs_sb_info *sbi = F2FS_SB(mapping->host->i_sb);
	unsigned int long flags;

	if (PageDirty(page)) {
		spin_lock_irqsave(&mapping->tree_lock, flags);
		radix_tree_tag_clear(&mapping->page_tree,
				page_index(page),
				PAGECACHE_TAG_DIRTY);
		spin_unlock_irqrestore(&mapping->tree_lock, flags);

		clear_page_dirty_for_io(page);
		dec_page_count(sbi, F2FS_DIRTY_NODES);
	}
	ClearPageUptodate(page);
}

static struct page *get_current_nat_page(struct f2fs_sb_info *sbi, nid_t nid)
{
	pgoff_t index = current_nat_addr(sbi, nid);
	return get_meta_page(sbi, index);
}

static struct page *get_next_nat_page(struct f2fs_sb_info *sbi, nid_t nid)
{
	struct page *src_page;
	struct page *dst_page;
	pgoff_t src_off;
	pgoff_t dst_off;
	void *src_addr;
	void *dst_addr;
	struct f2fs_nm_info *nm_i = NM_I(sbi);

	src_off = current_nat_addr(sbi, nid);
	dst_off = next_nat_addr(sbi, src_off);

	/* get current nat block page with lock */
	src_page = get_meta_page(sbi, src_off);

	/* Dirty src_page means that it is already the new target NAT page. */
	if (PageDirty(src_page))
		return src_page;

	dst_page = grab_meta_page(sbi, dst_off);

	src_addr = page_address(src_page);
	dst_addr = page_address(dst_page);
	memcpy(dst_addr, src_addr, PAGE_CACHE_SIZE);
	set_page_dirty(dst_page);
	f2fs_put_page(src_page, 1);

	set_to_next_nat(nm_i, nid);

	return dst_page;
}

/*
 * Readahead NAT pages
 */
static void ra_nat_pages(struct f2fs_sb_info *sbi, int nid)
{
	struct address_space *mapping = sbi->meta_inode->i_mapping;
	struct f2fs_nm_info *nm_i = NM_I(sbi);
	struct page *page;
	pgoff_t index;
	int i;

	for (i = 0; i < FREE_NID_PAGES; i++, nid += NAT_ENTRY_PER_BLOCK) {
		if (nid >= nm_i->max_nid)
			nid = 0;
		index = current_nat_addr(sbi, nid);

		page = grab_cache_page(mapping, index);
		if (!page)
			continue;
		if (f2fs_readpage(sbi, page, index, READ)) {
			f2fs_put_page(page, 1);
			continue;
		}
		page_cache_release(page);
	}
}

static struct nat_entry *__lookup_nat_cache(struct f2fs_nm_info *nm_i, nid_t n)
{
	return radix_tree_lookup(&nm_i->nat_root, n);
}

static unsigned int __gang_lookup_nat_cache(struct f2fs_nm_info *nm_i,
		nid_t start, unsigned int nr, struct nat_entry **ep)
{
	return radix_tree_gang_lookup(&nm_i->nat_root, (void **)ep, start, nr);
}

static void __del_from_nat_cache(struct f2fs_nm_info *nm_i, struct nat_entry *e)
{
	list_del(&e->list);
	radix_tree_delete(&nm_i->nat_root, nat_get_nid(e));
	nm_i->nat_cnt--;
	kmem_cache_free(nat_entry_slab, e);
}

int is_checkpointed_node(struct f2fs_sb_info *sbi, nid_t nid)
{
	struct f2fs_nm_info *nm_i = NM_I(sbi);
	struct nat_entry *e;
	int is_cp = 1;

	read_lock(&nm_i->nat_tree_lock);
	e = __lookup_nat_cache(nm_i, nid);
	if (e && !e->checkpointed)
		is_cp = 0;
	read_unlock(&nm_i->nat_tree_lock);
	return is_cp;
}

static struct nat_entry *grab_nat_entry(struct f2fs_nm_info *nm_i, nid_t nid)
{
	struct nat_entry *new;

	new = kmem_cache_alloc(nat_entry_slab, GFP_ATOMIC);
	if (!new)
		return NULL;
	if (radix_tree_insert(&nm_i->nat_root, nid, new)) {
		kmem_cache_free(nat_entry_slab, new);
		return NULL;
	}
	memset(new, 0, sizeof(struct nat_entry));
	nat_set_nid(new, nid);
	list_add_tail(&new->list, &nm_i->nat_entries);
	nm_i->nat_cnt++;
	return new;
}

static void cache_nat_entry(struct f2fs_nm_info *nm_i, nid_t nid,
						struct f2fs_nat_entry *ne)
{
	struct nat_entry *e;
retry:
	write_lock(&nm_i->nat_tree_lock);
	e = __lookup_nat_cache(nm_i, nid);
	if (!e) {
		e = grab_nat_entry(nm_i, nid);
		if (!e) {
			write_unlock(&nm_i->nat_tree_lock);
			goto retry;
		}
		nat_set_blkaddr(e, le32_to_cpu(ne->block_addr));
		nat_set_ino(e, le32_to_cpu(ne->ino));
		nat_set_version(e, ne->version);
		e->checkpointed = true;
	}
	write_unlock(&nm_i->nat_tree_lock);
}

static void set_node_addr(struct f2fs_sb_info *sbi, struct node_info *ni,
			block_t new_blkaddr)
{
	struct f2fs_nm_info *nm_i = NM_I(sbi);
	struct nat_entry *e;
retry:
	write_lock(&nm_i->nat_tree_lock);
	e = __lookup_nat_cache(nm_i, ni->nid);
	if (!e) {
		e = grab_nat_entry(nm_i, ni->nid);
		if (!e) {
			write_unlock(&nm_i->nat_tree_lock);
			goto retry;
		}
		e->ni = *ni;
		e->checkpointed = true;
		BUG_ON(ni->blk_addr == NEW_ADDR);
	} else if (new_blkaddr == NEW_ADDR) {
		/*
		 * when nid is reallocated,
		 * previous nat entry can be remained in nat cache.
		 * So, reinitialize it with new information.
		 */
		e->ni = *ni;
		BUG_ON(ni->blk_addr != NULL_ADDR);
	}

	if (new_blkaddr == NEW_ADDR)
		e->checkpointed = false;

	/* sanity check */
	BUG_ON(nat_get_blkaddr(e) != ni->blk_addr);
	BUG_ON(nat_get_blkaddr(e) == NULL_ADDR &&
			new_blkaddr == NULL_ADDR);
	BUG_ON(nat_get_blkaddr(e) == NEW_ADDR &&
			new_blkaddr == NEW_ADDR);
	BUG_ON(nat_get_blkaddr(e) != NEW_ADDR &&
			nat_get_blkaddr(e) != NULL_ADDR &&
			new_blkaddr == NEW_ADDR);

	/* increament version no as node is removed */
	if (nat_get_blkaddr(e) != NEW_ADDR && new_blkaddr == NULL_ADDR) {
		unsigned char version = nat_get_version(e);
		nat_set_version(e, inc_node_version(version));
	}

	/* change address */
	nat_set_blkaddr(e, new_blkaddr);
	__set_nat_cache_dirty(nm_i, e);
	write_unlock(&nm_i->nat_tree_lock);
}

static int try_to_free_nats(struct f2fs_sb_info *sbi, int nr_shrink)
{
	struct f2fs_nm_info *nm_i = NM_I(sbi);

	if (nm_i->nat_cnt < 2 * NM_WOUT_THRESHOLD)
		return 0;

	write_lock(&nm_i->nat_tree_lock);
	while (nr_shrink && !list_empty(&nm_i->nat_entries)) {
		struct nat_entry *ne;
		ne = list_first_entry(&nm_i->nat_entries,
					struct nat_entry, list);
		__del_from_nat_cache(nm_i, ne);
		nr_shrink--;
	}
	write_unlock(&nm_i->nat_tree_lock);
	return nr_shrink;
}

/*
 * This function returns always success
 */
void get_node_info(struct f2fs_sb_info *sbi, nid_t nid, struct node_info *ni)
{
	struct f2fs_nm_info *nm_i = NM_I(sbi);
	struct curseg_info *curseg = CURSEG_I(sbi, CURSEG_HOT_DATA);
	struct f2fs_summary_block *sum = curseg->sum_blk;
	nid_t start_nid = START_NID(nid);
	struct f2fs_nat_block *nat_blk;
	struct page *page = NULL;
	struct f2fs_nat_entry ne;
	struct nat_entry *e;
	int i;

	memset(&ne, 0, sizeof(struct f2fs_nat_entry));
	ni->nid = nid;

	/* Check nat cache */
	read_lock(&nm_i->nat_tree_lock);
	e = __lookup_nat_cache(nm_i, nid);
	if (e) {
		ni->ino = nat_get_ino(e);
		ni->blk_addr = nat_get_blkaddr(e);
		ni->version = nat_get_version(e);
	}
	read_unlock(&nm_i->nat_tree_lock);
	if (e)
		return;

	/* Check current segment summary */
	mutex_lock(&curseg->curseg_mutex);
	i = lookup_journal_in_cursum(sum, NAT_JOURNAL, nid, 0);
	if (i >= 0) {
		ne = nat_in_journal(sum, i);
		node_info_from_raw_nat(ni, &ne);
	}
	mutex_unlock(&curseg->curseg_mutex);
	if (i >= 0)
		goto cache;

	/* Fill node_info from nat page */
	page = get_current_nat_page(sbi, start_nid);
	nat_blk = (struct f2fs_nat_block *)page_address(page);
	ne = nat_blk->entries[nid - start_nid];
	node_info_from_raw_nat(ni, &ne);
	f2fs_put_page(page, 1);
cache:
	/* cache nat entry */
	cache_nat_entry(NM_I(sbi), nid, &ne);
}

/*
 * The maximum depth is four.
 * Offset[0] will have raw inode offset.
 */
static int get_node_path(long block, int offset[4], unsigned int noffset[4])
{
	const long direct_index = ADDRS_PER_INODE;
	const long direct_blks = ADDRS_PER_BLOCK;
	const long dptrs_per_blk = NIDS_PER_BLOCK;
	const long indirect_blks = ADDRS_PER_BLOCK * NIDS_PER_BLOCK;
	const long dindirect_blks = indirect_blks * NIDS_PER_BLOCK;
	int n = 0;
	int level = 0;

	noffset[0] = 0;

	if (block < direct_index) {
		offset[n++] = block;
		level = 0;
		goto got;
	}
	block -= direct_index;
	if (block < direct_blks) {
		offset[n++] = NODE_DIR1_BLOCK;
		noffset[n] = 1;
		offset[n++] = block;
		level = 1;
		goto got;
	}
	block -= direct_blks;
	if (block < direct_blks) {
		offset[n++] = NODE_DIR2_BLOCK;
		noffset[n] = 2;
		offset[n++] = block;
		level = 1;
		goto got;
	}
	block -= direct_blks;
	if (block < indirect_blks) {
		offset[n++] = NODE_IND1_BLOCK;
		noffset[n] = 3;
		offset[n++] = block / direct_blks;
		noffset[n] = 4 + offset[n - 1];
		offset[n++] = block % direct_blks;
		level = 2;
		goto got;
	}
	block -= indirect_blks;
	if (block < indirect_blks) {
		offset[n++] = NODE_IND2_BLOCK;
		noffset[n] = 4 + dptrs_per_blk;
		offset[n++] = block / direct_blks;
		noffset[n] = 5 + dptrs_per_blk + offset[n - 1];
		offset[n++] = block % direct_blks;
		level = 2;
		goto got;
	}
	block -= indirect_blks;
	if (block < dindirect_blks) {
		offset[n++] = NODE_DIND_BLOCK;
		noffset[n] = 5 + (dptrs_per_blk * 2);
		offset[n++] = block / indirect_blks;
		noffset[n] = 6 + (dptrs_per_blk * 2) +
			      offset[n - 1] * (dptrs_per_blk + 1);
		offset[n++] = (block / direct_blks) % dptrs_per_blk;
		noffset[n] = 7 + (dptrs_per_blk * 2) +
			      offset[n - 2] * (dptrs_per_blk + 1) +
			      offset[n - 1];
		offset[n++] = block % direct_blks;
		level = 3;
		goto got;
	} else {
		BUG();
	}
got:
	return level;
}

/*
 * Caller should call f2fs_put_dnode(dn).
 */
int get_dnode_of_data(struct dnode_of_data *dn, pgoff_t index, int ro)
{
	struct f2fs_sb_info *sbi = F2FS_SB(dn->inode->i_sb);
	struct page *npage[4];
	struct page *parent;
	int offset[4];
	unsigned int noffset[4];
	nid_t nids[4];
	int level, i;
	int err = 0;

	level = get_node_path(index, offset, noffset);

	nids[0] = dn->inode->i_ino;
	npage[0] = get_node_page(sbi, nids[0]);
	if (IS_ERR(npage[0]))
		return PTR_ERR(npage[0]);

	parent = npage[0];
	nids[1] = get_nid(parent, offset[0], true);
	dn->inode_page = npage[0];
	dn->inode_page_locked = true;

	/* get indirect or direct nodes */
	for (i = 1; i <= level; i++) {
		bool done = false;

		if (!nids[i] && !ro) {
			mutex_lock_op(sbi, NODE_NEW);

			/* alloc new node */
			if (!alloc_nid(sbi, &(nids[i]))) {
				mutex_unlock_op(sbi, NODE_NEW);
				err = -ENOSPC;
				goto release_pages;
			}

			dn->nid = nids[i];
			npage[i] = new_node_page(dn, noffset[i]);
			if (IS_ERR(npage[i])) {
				alloc_nid_failed(sbi, nids[i]);
				mutex_unlock_op(sbi, NODE_NEW);
				err = PTR_ERR(npage[i]);
				goto release_pages;
			}

			set_nid(parent, offset[i - 1], nids[i], i == 1);
			alloc_nid_done(sbi, nids[i]);
			mutex_unlock_op(sbi, NODE_NEW);
			done = true;
		} else if (ro && i == level && level > 1) {
			npage[i] = get_node_page_ra(parent, offset[i - 1]);
			if (IS_ERR(npage[i])) {
				err = PTR_ERR(npage[i]);
				goto release_pages;
			}
			done = true;
		}
		if (i == 1) {
			dn->inode_page_locked = false;
			unlock_page(parent);
		} else {
			f2fs_put_page(parent, 1);
		}

		if (!done) {
			npage[i] = get_node_page(sbi, nids[i]);
			if (IS_ERR(npage[i])) {
				err = PTR_ERR(npage[i]);
				f2fs_put_page(npage[0], 0);
				goto release_out;
			}
		}
		if (i < level) {
			parent = npage[i];
			nids[i + 1] = get_nid(parent, offset[i], false);
		}
	}
	dn->nid = nids[level];
	dn->ofs_in_node = offset[level];
	dn->node_page = npage[level];
	dn->data_blkaddr = datablock_addr(dn->node_page, dn->ofs_in_node);
	return 0;

release_pages:
	f2fs_put_page(parent, 1);
	if (i > 1)
		f2fs_put_page(npage[0], 0);
release_out:
	dn->inode_page = NULL;
	dn->node_page = NULL;
	return err;
}

static void truncate_node(struct dnode_of_data *dn)
{
	struct f2fs_sb_info *sbi = F2FS_SB(dn->inode->i_sb);
	struct node_info ni;

	get_node_info(sbi, dn->nid, &ni);
	if (dn->inode->i_blocks == 0) {
		BUG_ON(ni.blk_addr != NULL_ADDR);
		goto invalidate;
	}
	BUG_ON(ni.blk_addr == NULL_ADDR);

	/* Deallocate node address */
	invalidate_blocks(sbi, ni.blk_addr);
	dec_valid_node_count(sbi, dn->inode, 1);
	set_node_addr(sbi, &ni, NULL_ADDR);

	if (dn->nid == dn->inode->i_ino) {
		remove_orphan_inode(sbi, dn->nid);
		dec_valid_inode_count(sbi);
	} else {
		sync_inode_page(dn);
	}
invalidate:
	clear_node_page_dirty(dn->node_page);
	F2FS_SET_SB_DIRT(sbi);

	f2fs_put_page(dn->node_page, 1);
	dn->node_page = NULL;
}

static int truncate_dnode(struct dnode_of_data *dn)
{
	struct f2fs_sb_info *sbi = F2FS_SB(dn->inode->i_sb);
	struct page *page;

	if (dn->nid == 0)
		return 1;

	/* get direct node */
	page = get_node_page(sbi, dn->nid);
	if (IS_ERR(page) && PTR_ERR(page) == -ENOENT)
		return 1;
	else if (IS_ERR(page))
		return PTR_ERR(page);

	/* Make dnode_of_data for parameter */
	dn->node_page = page;
	dn->ofs_in_node = 0;
	truncate_data_blocks(dn);
	truncate_node(dn);
	return 1;
}

static int truncate_nodes(struct dnode_of_data *dn, unsigned int nofs,
						int ofs, int depth)
{
	struct f2fs_sb_info *sbi = F2FS_SB(dn->inode->i_sb);
	struct dnode_of_data rdn = *dn;
	struct page *page;
	struct f2fs_node *rn;
	nid_t child_nid;
	unsigned int child_nofs;
	int freed = 0;
	int i, ret;

	if (dn->nid == 0)
		return NIDS_PER_BLOCK + 1;

	page = get_node_page(sbi, dn->nid);
	if (IS_ERR(page))
		return PTR_ERR(page);

	rn = (struct f2fs_node *)page_address(page);
	if (depth < 3) {
		for (i = ofs; i < NIDS_PER_BLOCK; i++, freed++) {
			child_nid = le32_to_cpu(rn->in.nid[i]);
			if (child_nid == 0)
				continue;
			rdn.nid = child_nid;
			ret = truncate_dnode(&rdn);
			if (ret < 0)
				goto out_err;
			set_nid(page, i, 0, false);
		}
	} else {
		child_nofs = nofs + ofs * (NIDS_PER_BLOCK + 1) + 1;
		for (i = ofs; i < NIDS_PER_BLOCK; i++) {
			child_nid = le32_to_cpu(rn->in.nid[i]);
			if (child_nid == 0) {
				child_nofs += NIDS_PER_BLOCK + 1;
				continue;
			}
			rdn.nid = child_nid;
			ret = truncate_nodes(&rdn, child_nofs, 0, depth - 1);
			if (ret == (NIDS_PER_BLOCK + 1)) {
				set_nid(page, i, 0, false);
				child_nofs += ret;
			} else if (ret < 0 && ret != -ENOENT) {
				goto out_err;
			}
		}
		freed = child_nofs;
	}

	if (!ofs) {
		/* remove current indirect node */
		dn->node_page = page;
		truncate_node(dn);
		freed++;
	} else {
		f2fs_put_page(page, 1);
	}
	return freed;

out_err:
	f2fs_put_page(page, 1);
	return ret;
}

static int truncate_partial_nodes(struct dnode_of_data *dn,
			struct f2fs_inode *ri, int *offset, int depth)
{
	struct f2fs_sb_info *sbi = F2FS_SB(dn->inode->i_sb);
	struct page *pages[2];
	nid_t nid[3];
	nid_t child_nid;
	int err = 0;
	int i;
	int idx = depth - 2;

	nid[0] = le32_to_cpu(ri->i_nid[offset[0] - NODE_DIR1_BLOCK]);
	if (!nid[0])
		return 0;

	/* get indirect nodes in the path */
	for (i = 0; i < depth - 1; i++) {
		/* refernece count'll be increased */
		pages[i] = get_node_page(sbi, nid[i]);
		if (IS_ERR(pages[i])) {
			depth = i + 1;
			err = PTR_ERR(pages[i]);
			goto fail;
		}
		nid[i + 1] = get_nid(pages[i], offset[i + 1], false);
	}

	/* free direct nodes linked to a partial indirect node */
	for (i = offset[depth - 1]; i < NIDS_PER_BLOCK; i++) {
		child_nid = get_nid(pages[idx], i, false);
		if (!child_nid)
			continue;
		dn->nid = child_nid;
		err = truncate_dnode(dn);
		if (err < 0)
			goto fail;
		set_nid(pages[idx], i, 0, false);
	}

	if (offset[depth - 1] == 0) {
		dn->node_page = pages[idx];
		dn->nid = nid[idx];
		truncate_node(dn);
	} else {
		f2fs_put_page(pages[idx], 1);
	}
	offset[idx]++;
	offset[depth - 1] = 0;
fail:
	for (i = depth - 3; i >= 0; i--)
		f2fs_put_page(pages[i], 1);
	return err;
}

/*
 * All the block addresses of data and nodes should be nullified.
 */
int truncate_inode_blocks(struct inode *inode, pgoff_t from)
{
	struct f2fs_sb_info *sbi = F2FS_SB(inode->i_sb);
	int err = 0, cont = 1;
	int level, offset[4], noffset[4];
	unsigned int nofs;
	struct f2fs_node *rn;
	struct dnode_of_data dn;
	struct page *page;

	level = get_node_path(from, offset, noffset);

	page = get_node_page(sbi, inode->i_ino);
	if (IS_ERR(page))
		return PTR_ERR(page);

	set_new_dnode(&dn, inode, page, NULL, 0);
	unlock_page(page);

	rn = page_address(page);
	switch (level) {
	case 0:
	case 1:
		nofs = noffset[1];
		break;
	case 2:
		nofs = noffset[1];
		if (!offset[level - 1])
			goto skip_partial;
		err = truncate_partial_nodes(&dn, &rn->i, offset, level);
		if (err < 0 && err != -ENOENT)
			goto fail;
		nofs += 1 + NIDS_PER_BLOCK;
		break;
	case 3:
		nofs = 5 + 2 * NIDS_PER_BLOCK;
		if (!offset[level - 1])
			goto skip_partial;
		err = truncate_partial_nodes(&dn, &rn->i, offset, level);
		if (err < 0 && err != -ENOENT)
			goto fail;
		break;
	default:
		BUG();
	}

skip_partial:
	while (cont) {
		dn.nid = le32_to_cpu(rn->i.i_nid[offset[0] - NODE_DIR1_BLOCK]);
		switch (offset[0]) {
		case NODE_DIR1_BLOCK:
		case NODE_DIR2_BLOCK:
			err = truncate_dnode(&dn);
			break;

		case NODE_IND1_BLOCK:
		case NODE_IND2_BLOCK:
			err = truncate_nodes(&dn, nofs, offset[1], 2);
			break;

		case NODE_DIND_BLOCK:
			err = truncate_nodes(&dn, nofs, offset[1], 3);
			cont = 0;
			break;

		default:
			BUG();
		}
		if (err < 0 && err != -ENOENT)
			goto fail;
		if (offset[1] == 0 &&
				rn->i.i_nid[offset[0] - NODE_DIR1_BLOCK]) {
			lock_page(page);
			wait_on_page_writeback(page);
			rn->i.i_nid[offset[0] - NODE_DIR1_BLOCK] = 0;
			set_page_dirty(page);
			unlock_page(page);
		}
		offset[1] = 0;
		offset[0]++;
		nofs += err;
	}
fail:
	f2fs_put_page(page, 0);
	return err > 0 ? 0 : err;
}

int remove_inode_page(struct inode *inode)
{
	struct f2fs_sb_info *sbi = F2FS_SB(inode->i_sb);
	struct page *page;
	nid_t ino = inode->i_ino;
	struct dnode_of_data dn;

	mutex_lock_op(sbi, NODE_TRUNC);
	page = get_node_page(sbi, ino);
	if (IS_ERR(page)) {
		mutex_unlock_op(sbi, NODE_TRUNC);
		return PTR_ERR(page);
	}

	if (F2FS_I(inode)->i_xattr_nid) {
		nid_t nid = F2FS_I(inode)->i_xattr_nid;
		struct page *npage = get_node_page(sbi, nid);

		if (IS_ERR(npage)) {
			mutex_unlock_op(sbi, NODE_TRUNC);
			return PTR_ERR(npage);
		}

		F2FS_I(inode)->i_xattr_nid = 0;
		set_new_dnode(&dn, inode, page, npage, nid);
		dn.inode_page_locked = 1;
		truncate_node(&dn);
	}

	/* 0 is possible, after f2fs_new_inode() is failed */
	BUG_ON(inode->i_blocks != 0 && inode->i_blocks != 1);
	set_new_dnode(&dn, inode, page, page, ino);
	truncate_node(&dn);

	mutex_unlock_op(sbi, NODE_TRUNC);
	return 0;
}

int new_inode_page(struct inode *inode, struct dentry *dentry)
{
	struct f2fs_sb_info *sbi = F2FS_SB(inode->i_sb);
	struct page *page;
	struct dnode_of_data dn;

	/* allocate inode page for new inode */
	set_new_dnode(&dn, inode, NULL, NULL, inode->i_ino);
	mutex_lock_op(sbi, NODE_NEW);
	page = new_node_page(&dn, 0);
	init_dent_inode(dentry, page);
	mutex_unlock_op(sbi, NODE_NEW);
	if (IS_ERR(page))
		return PTR_ERR(page);
	f2fs_put_page(page, 1);
	return 0;
}

struct page *new_node_page(struct dnode_of_data *dn, unsigned int ofs)
{
	struct f2fs_sb_info *sbi = F2FS_SB(dn->inode->i_sb);
	struct address_space *mapping = sbi->node_inode->i_mapping;
	struct node_info old_ni, new_ni;
	struct page *page;
	int err;

	if (is_inode_flag_set(F2FS_I(dn->inode), FI_NO_ALLOC))
		return ERR_PTR(-EPERM);

	page = grab_cache_page(mapping, dn->nid);
	if (!page)
		return ERR_PTR(-ENOMEM);

	get_node_info(sbi, dn->nid, &old_ni);

	SetPageUptodate(page);
	fill_node_footer(page, dn->nid, dn->inode->i_ino, ofs, true);

	/* Reinitialize old_ni with new node page */
	BUG_ON(old_ni.blk_addr != NULL_ADDR);
	new_ni = old_ni;
	new_ni.ino = dn->inode->i_ino;

	if (!inc_valid_node_count(sbi, dn->inode, 1)) {
		err = -ENOSPC;
		goto fail;
	}
	set_node_addr(sbi, &new_ni, NEW_ADDR);
	set_cold_node(dn->inode, page);

	dn->node_page = page;
	sync_inode_page(dn);
	set_page_dirty(page);
	if (ofs == 0)
		inc_valid_inode_count(sbi);

	return page;

fail:
	clear_node_page_dirty(page);
	f2fs_put_page(page, 1);
	return ERR_PTR(err);
}

static int read_node_page(struct page *page, int type)
{
	struct f2fs_sb_info *sbi = F2FS_SB(page->mapping->host->i_sb);
	struct node_info ni;

	get_node_info(sbi, page->index, &ni);

	if (ni.blk_addr == NULL_ADDR)
		return -ENOENT;
	return f2fs_readpage(sbi, page, ni.blk_addr, type);
}

/*
 * Readahead a node page
 */
void ra_node_page(struct f2fs_sb_info *sbi, nid_t nid)
{
	struct address_space *mapping = sbi->node_inode->i_mapping;
	struct page *apage;

	apage = find_get_page(mapping, nid);
	if (apage && PageUptodate(apage))
		goto release_out;
	f2fs_put_page(apage, 0);

	apage = grab_cache_page(mapping, nid);
	if (!apage)
		return;

	if (read_node_page(apage, READA))
		goto unlock_out;

	page_cache_release(apage);
	return;

unlock_out:
	unlock_page(apage);
release_out:
	page_cache_release(apage);
}

struct page *get_node_page(struct f2fs_sb_info *sbi, pgoff_t nid)
{
	int err;
	struct page *page;
	struct address_space *mapping = sbi->node_inode->i_mapping;

	page = grab_cache_page(mapping, nid);
	if (!page)
		return ERR_PTR(-ENOMEM);

	err = read_node_page(page, READ_SYNC);
	if (err) {
		f2fs_put_page(page, 1);
		return ERR_PTR(err);
	}

	BUG_ON(nid != nid_of_node(page));
	mark_page_accessed(page);
	return page;
}

/*
 * Return a locked page for the desired node page.
 * And, readahead MAX_RA_NODE number of node pages.
 */
struct page *get_node_page_ra(struct page *parent, int start)
{
	struct f2fs_sb_info *sbi = F2FS_SB(parent->mapping->host->i_sb);
	struct address_space *mapping = sbi->node_inode->i_mapping;
	int i, end;
	int err = 0;
	nid_t nid;
	struct page *page;

	/* First, try getting the desired direct node. */
	nid = get_nid(parent, start, false);
	if (!nid)
		return ERR_PTR(-ENOENT);

	page = find_get_page(mapping, nid);
	if (page && PageUptodate(page))
		goto page_hit;
	f2fs_put_page(page, 0);

repeat:
	page = grab_cache_page(mapping, nid);
	if (!page)
		return ERR_PTR(-ENOMEM);

	err = read_node_page(page, READA);
	if (err) {
		f2fs_put_page(page, 1);
		return ERR_PTR(err);
	}

	/* Then, try readahead for siblings of the desired node */
	end = start + MAX_RA_NODE;
	end = min(end, NIDS_PER_BLOCK);
	for (i = start + 1; i < end; i++) {
		nid = get_nid(parent, i, false);
		if (!nid)
			continue;
		ra_node_page(sbi, nid);
	}

page_hit:
	lock_page(page);
	if (PageError(page)) {
		f2fs_put_page(page, 1);
		return ERR_PTR(-EIO);
	}

	/* Has the page been truncated? */
	if (page->mapping != mapping) {
		f2fs_put_page(page, 1);
		goto repeat;
	}
	return page;
}

void sync_inode_page(struct dnode_of_data *dn)
{
	if (IS_INODE(dn->node_page) || dn->inode_page == dn->node_page) {
		update_inode(dn->inode, dn->node_page);
	} else if (dn->inode_page) {
		if (!dn->inode_page_locked)
			lock_page(dn->inode_page);
		update_inode(dn->inode, dn->inode_page);
		if (!dn->inode_page_locked)
			unlock_page(dn->inode_page);
	} else {
		f2fs_write_inode(dn->inode, NULL);
	}
}

int sync_node_pages(struct f2fs_sb_info *sbi, nid_t ino,
					struct writeback_control *wbc)
{
	struct address_space *mapping = sbi->node_inode->i_mapping;
	pgoff_t index, end;
	struct pagevec pvec;
	int step = ino ? 2 : 0;
	int nwritten = 0, wrote = 0;

	pagevec_init(&pvec, 0);

next_step:
	index = 0;
	end = LONG_MAX;

	while (index <= end) {
		int i, nr_pages;
		nr_pages = pagevec_lookup_tag(&pvec, mapping, &index,
				PAGECACHE_TAG_DIRTY,
				min(end - index, (pgoff_t)PAGEVEC_SIZE-1) + 1);
		if (nr_pages == 0)
			break;

		for (i = 0; i < nr_pages; i++) {
			struct page *page = pvec.pages[i];

			/*
			 * flushing sequence with step:
			 * 0. indirect nodes
			 * 1. dentry dnodes
			 * 2. file dnodes
			 */
			if (step == 0 && IS_DNODE(page))
				continue;
			if (step == 1 && (!IS_DNODE(page) ||
						is_cold_node(page)))
				continue;
			if (step == 2 && (!IS_DNODE(page) ||
						!is_cold_node(page)))
				continue;

			/*
			 * If an fsync mode,
			 * we should not skip writing node pages.
			 */
			if (ino && ino_of_node(page) == ino)
				lock_page(page);
			else if (!trylock_page(page))
				continue;

			if (unlikely(page->mapping != mapping)) {
continue_unlock:
				unlock_page(page);
				continue;
			}
			if (ino && ino_of_node(page) != ino)
				goto continue_unlock;

			if (!PageDirty(page)) {
				/* someone wrote it for us */
				goto continue_unlock;
			}

			if (!clear_page_dirty_for_io(page))
				goto continue_unlock;

			/* called by fsync() */
			if (ino && IS_DNODE(page)) {
				int mark = !is_checkpointed_node(sbi, ino);
				set_fsync_mark(page, 1);
				if (IS_INODE(page))
					set_dentry_mark(page, mar S                 ¯~ü                                 s®ü                 s®ü                 †ü                                 sàü                 sàü                 _                                 sàü                 ë†                 ë†                 ë†                                 }Äü                 sàü                 ë†                 ë†                 sàü                 _                 ë†                                 sàü                 ë†                 ë†                 ë†                                 }Äü                 sàü                 ë†                 ë†                 sàü                 _                 ë†                                 1ü                 1ü                 1ü                                 ë†                 ë†                                 ë†#                                 sàü                 sàü                                 S                 S                                 s®ü                                 }Äü                 sàü                 ë†                 ë†                 sàü                 _                 ë†                                 }Äü                 sàü                 ë†                 ë†                 sàü                 _                 ë†                                 1ü                                 P                 P                                 %ü                                 }Äü                                 }Äü                                 }Äü                 sàü                 ë†                 ë†                 sàü                 _                 ë†                                 }Äü                                 }Äü                 sàü                 ë†                 ë†                 sàü                 _                 ë†                                 1ü                                 P                                 #ü                                 U                 \                 |‡~ü                                 T                 _                                 0ü                 ]                 1ü                                 P                 S                 P                                	 pÄ
 ü                 p 
 ü                 qÄ~ü                                 p                  Q                                 |†                 |                                  pÄ                                 ë®#àü                
         ü                                 ë®#àü                
         ü                                 1ü                                 pàü                                 ë®#®ü                 ë®#®ü                                 ë®#àü                
         ü                                 ë®#àü                
         ü                                 1ü                                 ë®#àü                                 ë®#àü                                 ë®#àü                
         ü                                 ë®#àü                                 ë®#àü                
         ü                                 1ü                                 ^                                 ë®#ü                                 ë®#àü                
         ü                                 ë®#àü                
         ü                                 U                 \                 |‡~ü                 |‡~ü                                 ^                 S                 ^                 ^                                	 ~Ä
 ü                 p 
 ü                                 |†                 |                                  ~Ä                                 p                                  ~®ü                 ~®ü                 ~®ü                                 ~àü                 ~àü                                
         ü                 ~àü                                
         ü                 ~àü                                 1ü                                
         ü                                
         ü                                
         ü                 ~àü                                
         ü                                
         ü                 ~àü                                 1ü                                
         ü                 ~àü                                
         ü                 ~àü                                 U                 \                 \                                 T                 S                                 t»ü                 s»ü                 s»ü                                 t®ü                 s®ü                 s®ü                                 T                 S                 S                                 T                 S                 S                                 t∞ü                 s∞ü                 s∞ü                                 s¨ü                                 s»ü                                 P                                 U                 \                 U                 \                                 P                 S                 P                 S                 U                 S                 T                                 U                                 P                 S                 P                                 pàü                 sàü                                 sàü                                 sàü                                 sàü                                 sàü                                 sàü                                 1ü                                 P                                 U                                 P                 S                 P                                 U                 ë∞                                 T                 ë¨                                 Q                 S                                 R                 \                                 X                 _                                 	ˇü                 P                 pü                 P                                 0ü                 ]                                 ë∏                                 P                 pü                                 P                 pü                 P                                 ]                                 ë∏                                 P                 pü                                 	ˇü                 P                 pü                 P                                 0ü                 \                                 P                 pü                                 S                                 P                 pü                 P                                 S                                 P                 pü                                 ^                                 U                 S                                 T                 \                 \                                 ]                 ]                                 P                 S                 P                 S                                 P                 S                 S                                 2ü                 2ü                                
         ü                
         ü                                 3ü                 3ü                                 U                 U                                 U                 U                                 3ü                 3ü                                 U                                 T                                 Q                                 R                                 P                 S                 P                 S                 P                                 P                 S                 S                 P                                 2ü                 2ü                                
         ü                
         ü                                 3ü                 3ü                                 U                 U                                 U                 U                                 3ü                 3ü                                 U                 S                 S                                 T                                 Q                 _                 _                                 R                 ë∏                                 X                 ^                 ^                                 ]                 ]                                 P                 \                 P                 \                 P                 S                 \                                
         ü                 ë∞#àü                
         ü                                
         ü                 ë∞#àü                
         ü                                
         ü                 ë∞#àü                
         ü                                
         ü                 ë∞#àü                
         ü                                 P                 \                                 2ü                                
         ü                                 3ü                                 U                                 U                 U                                 3ü                 3ü                                 P                 S                 P                 S                                
         ü                
         ü                                
         ü                                
         ü                                
         ü                 ë∞#àü                
         ü                                
         ü                                
         ü                 ë∞#àü                
         ü                                 1ü                                 S                                 2ü                                
         ü                                 3ü                                 U                                 U                 U                                 3ü                 3ü                                 ë∞#àü                                 ]                                 ë∞#–ü                                 }                                  ë∞#àü                                 ë∞#àü                
         ü                                
         ü                 ë∞#àü                
         ü                                 ë∞#àü                
         ü                                
         ü                 ë∞#àü                
         ü                                 1ü                                
         ü                                
         ü                                
         ü                 ë∞#àü                
         ü                                
         ü                                
         ü                 ë∞#àü                
         ü                                 1ü                                
         ü                 ë∞#àü                
         ü                                
         ü                 ë∞#àü                
         ü                                 U                 \                 \                                 T                 ^                 ^                                 ]                 ]                                 P                 S                 P                 S                 P                 S                 U                 S                                
         ü                 sàü                
         ü                                
         ü                 sàü                
         ü                                
         ü                 sàü                
         ü                                
         ü                 sàü                
         ü                                 P                 S                                 2ü                                
         ü                                 3ü                                 U                                 U                 U                                 3ü                 3ü                                 P                 S                 P                 \                                
         ü                
         ü                                
         ü                                
         ü                                
         ü                 sàü                
         ü                                
         ü                                
         ü                 sàü                
         ü                                 1ü                                 S                                 2ü                                
         ü                                 3ü                                 U                                 U                 U                                 3ü                 3ü                                 U                                 }                                  sàü                
         ü                                
         ü                 sàü                
         ü                                 sàü                
         ü                                
         ü                 sàü                
         ü                                
         ü                 sàü                
         ü                                
         ü                 sàü                
         ü                ]       0D cº  clear_nlink ÿº  set_nlink ﬂΩ  __iget væ  generic_delete_inode ©æ  bmap ˛æ  inode_needs_sync 1ø  inode_init_owner ¿  inode_dio_done ä¿  inode_wait ø¿  inode_dio_wait –¡  inode_owner_or_capable 9¬  init_special_inode ä¬  inode_init ª¬  inode_init_early ˙À  ilookup5_nowait ãÕ  igrab ≥œ  iunique 1”  __remove_inode_hash ÷  __insert_inode_hash ¯Ÿ  file_update_time ø⁄  get_next_ino ]€  should_remove_suid ≤€  file_remove_suid }‹  touch_atime Í›  unlock_new_inode ﬂ  ihold ßﬂ  inc_nlink D‡  drop_nlink ƒ‚  free_inode_nonrcu í„  inode_sb_list_add ˝‰  inode_add_lru °Ê  clear_inode Ë  address_space_init_once ¨Ë  inode_init_once ãÈ  __destroy_inode ¥Ó  iput ‰Ò  insert_inode_locked4 å˜  insert_inode_locked Ì˝  prune_icache_sb Î invalidate_inodes k evict_inodes ˘ inode_init_always " new_inode_pseudo t new_inode 
 proc_nr_inodes I get_nr_dirty_inodes B ilookup : ilookup5 / iget5_locked ¶ iget_locked v? inodes_stat ¶? empty_aops YA inode_sb_list_lock |A __pcpu_unique_nr_inodes íA nr_inodes ®A __pcpu_unique_nr_unused æA nr_unused ïB __pcpu_unique_last_ino ¨B last_ino     .       0D 1   kernel_symbol Ü   __s8 ò   __u8 ™   __s16 º   __u16 Œ   __s32 ‡   __u32 Î   __s64 ˝   __u64   s8   u8 #  s16 .  u16 9  s32 D  u32 O  s64 Z  u64 ú  __kernel_long_t Æ  __kernel_ulong_t π  __kernel_ino_t ƒ  __kernel_pid_t œ  __kernel_uid32_t ⁄  __kernel_gid32_t Â  __kernel_size_t   __kernel_ssize_t ˚  __kernel_loff_t   __kernel_time_t   __kernel_clock_t   __kernel_timer_t '  __kernel_clockid_t 8  __kernel_dev_t C  dev_t N  ino_t Y  umode_t d  pid_t o  clockid_t z  bool å  uid_t ó  gid_t ¢  loff_t ≠  size_t ∏  ssize_t √  time_t Œ  int32_t Ÿ  uint32_t ‰  sector_t Ô  blkcnt_t ˙  gfp_t   fmode_t   oom_flags_t   phys_addr_t &  resource_size_t F  atomic_t f  atomic64_t q  list_head ú  hlist_head µ  hlist_node ˆ  callback_head P  obs_kernel_param »  pt_regs Ω  desc_struct –  gate_struct64 b  gate_desc m  desc_ptr í  pteval_t ù  pmdval_t ®  pudval_t ≥  pgdval_t æ  pgprotval_t ﬁ  pte_t È  pgprot   pgprot_t "  pgd_t B  pud_t b  pmd_t m  pgtable_t ∂  paravirt_callee_save œ  pv_info 	  pv_lazy_ops =	  pv_time_ops ú	  pv_cpu_ops 9  pv_irq_ops ö  pv_apic_ops œ  pv_mmu_ops Ï  kernel_vm86_regs ì  math_emu_info ;  cpumask “  cpumask_t ›  cpumask_var_t    cpuinfo_x86 ë!  x86_hw_tss '  tss_struct ;"  i387_fsave_struct w#  i387_fxsave_struct *$  i387_soft_struct %  ymmh_struct 1%  xsave_hdr_struct Ü%  xsave_struct æ%  thread_xstate ˝%  fpu l&  irq_stack_union Y  thread_struct …&  atomic_long_t ‘&  __ticket_t ﬂ&  __ticketpair_t Í&  __raw_tickets …  arch_spinlock .'  arch_spinlock_t s'  arch_rwlock_t ~'  lock_class_key á'  raw_spinlock †'  raw_spinlock_t ø'  spinlock “'  spinlock_t Ú'  rwlock_t ˝'  wait_queue_t E(  wait_queue_func_t (  __wait_queue {(  wait_bit_key †(  wait_bit_queue ≈(  __wait_queue_head Í(  wait_queue_head_t )  seqlock_t !)  se