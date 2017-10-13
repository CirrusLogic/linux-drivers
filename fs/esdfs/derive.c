/*
 * Copyright (c) 2013-2014 Motorola Mobility LLC
 * Copyright (C) 2017      Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/proc_fs.h>
#include <linux/hashtable.h>
#include <linux/syscalls.h>
#include <linux/fcntl.h>
#include <linux/ctype.h>
#include <linux/vmalloc.h>
#include <linux/uaccess.h>
#include "esdfs.h"

static char *names_secure[] = {
	"autorun.inf",
	".android_secure",
	"android_secure",
	"" };

/* special path name searches */
static inline bool match_name(struct qstr *name, char *names[])
{
	int i = 0;

	BUG_ON(!name);
	for (i = 0; *names[i]; i++)
		if (name->len == strlen(names[i]) &&
		    !strncasecmp(names[i],
				 name->name,
				 name->len))
			return true;

	return false;
}

static inline uid_t derive_uid(struct esdfs_inode_info *inode_i, uid_t uid)
{
	return inode_i->userid * PKG_APPID_PER_USER +
	       (uid % PKG_APPID_PER_USER);
}

unsigned esdfs_package_list_version;

static void fixup_perms_by_flag(int flags, const struct qstr *key,
					uint32_t userid)
{
	esdfs_package_list_version++;
}

static struct pkg_list esdfs_pkg_list = {
		.update = fixup_perms_by_flag,
};

int esdfs_init_package_list(void)
{
	pkglist_register_update_listener(&esdfs_pkg_list);
	return 0;
}

void esdfs_destroy_package_list(void)
{
	pkglist_unregister_update_listener(&esdfs_pkg_list);
}

/*
 * Derive an entry's premissions tree position based on its parent.
 */
void esdfs_derive_perms(struct dentry *dentry)
{
	struct esdfs_inode_info *inode_i = ESDFS_I(dentry->d_inode);
	bool is_root;
	int ret;
	kuid_t appid;

	spin_lock(&dentry->d_lock);
	is_root = IS_ROOT(dentry);
	spin_unlock(&dentry->d_lock);
	if (is_root)
		return;

	/* Inherit from the parent to start */
	inode_i->tree = ESDFS_I(dentry->d_parent->d_inode)->tree;
	inode_i->userid = ESDFS_I(dentry->d_parent->d_inode)->userid;
	inode_i->appid = ESDFS_I(dentry->d_parent->d_inode)->appid;

	/*
	 * ESDFS_TREE_MEDIA* are intentionally dead ends.
	 */
	switch (inode_i->tree) {
	case ESDFS_TREE_ROOT_LEGACY:
		inode_i->tree = ESDFS_TREE_ROOT;
		ret = kstrtou32(dentry->d_name.name, 0, &inode_i->userid);
		if (!strncasecmp(dentry->d_name.name,
					"obb",
					dentry->d_name.len))
			inode_i->tree = ESDFS_TREE_ANDROID_OBB;
		break;

	case ESDFS_TREE_ROOT:
		inode_i->tree = ESDFS_TREE_MEDIA;
		if (!strncasecmp(dentry->d_name.name,
				 "Android",
				 dentry->d_name.len))
			inode_i->tree = ESDFS_TREE_ANDROID;
		break;

	case ESDFS_TREE_ANDROID:
		if (!strncasecmp(dentry->d_name.name,
				 "data",
				 dentry->d_name.len))
			inode_i->tree = ESDFS_TREE_ANDROID_DATA;
		else if (!strncasecmp(dentry->d_name.name,
					"obb",
					dentry->d_name.len))
			inode_i->tree = ESDFS_TREE_ANDROID_OBB;
		else if (!strncasecmp(dentry->d_name.name,
					"media",
					dentry->d_name.len))
			inode_i->tree = ESDFS_TREE_ANDROID_MEDIA;
		else if (ESDFS_RESTRICT_PERMS(ESDFS_SB(dentry->d_sb)) &&
			 !strncasecmp(dentry->d_name.name,
					"user",
					dentry->d_name.len))
			inode_i->tree = ESDFS_TREE_ANDROID_USER;
		break;

	case ESDFS_TREE_ANDROID_DATA:
	case ESDFS_TREE_ANDROID_OBB:
	case ESDFS_TREE_ANDROID_MEDIA:
		appid = pkglist_get_allowed_appid(dentry->d_name.name,
						inode_i->userid);
		if (uid_valid(appid))
			inode_i->appid = appid.val;
		else
			inode_i->appid = 0;
		inode_i->tree = ESDFS_TREE_ANDROID_APP;
		break;

	case ESDFS_TREE_ANDROID_USER:
		/* Another user, so start over */
		inode_i->tree = ESDFS_TREE_ROOT;
		ret = kstrtou32(dentry->d_name.name, 0, &inode_i->userid);
		break;
	}
}

/* Apply tree position-specific permissions */
void esdfs_set_derived_perms(struct inode *inode)
{
	struct esdfs_sb_info *sbi = ESDFS_SB(inode->i_sb);
	struct esdfs_inode_info *inode_i = ESDFS_I(inode);
	gid_t gid = sbi->upper_perms.gid;

	i_uid_write(inode, sbi->upper_perms.uid);
	inode->i_mode &= S_IFMT;
	if (ESDFS_RESTRICT_PERMS(sbi))
		i_gid_write(inode, gid);
	else {
		if (gid == AID_SDCARD_RW)
			i_gid_write(inode, AID_SDCARD_RW);
		else
			i_gid_write(inode, derive_uid(inode_i, gid));
		inode->i_mode |= sbi->upper_perms.dmask;
	}

	switch (inode_i->tree) {
	case ESDFS_TREE_ROOT_LEGACY:
		if (ESDFS_RESTRICT_PERMS(sbi))
			inode->i_mode |= sbi->upper_perms.dmask;
		else if (test_opt(sbi, DERIVE_MULTI)) {
			inode->i_mode &= S_IFMT;
			inode->i_mode |= 0711;
		}
		break;

	case ESDFS_TREE_NONE:
	case ESDFS_TREE_ROOT:
		if (ESDFS_RESTRICT_PERMS(sbi)) {
			i_gid_write(inode, AID_SDCARD_R);
			inode->i_mode |= sbi->upper_perms.dmask;
		} else if (test_opt(sbi, DERIVE_PUBLIC) &&
			   test_opt(ESDFS_SB(inode->i_sb), DERIVE_CONFINE)) {
			inode->i_mode &= S_IFMT;
			inode->i_mode |= 0771;
		}
		break;

	case ESDFS_TREE_MEDIA:
		if (ESDFS_RESTRICT_PERMS(sbi)) {
			i_gid_write(inode, AID_SDCARD_R);
			inode->i_mode |= 0770;
		}
		break;

	case ESDFS_TREE_ANDROID:
	case ESDFS_TREE_ANDROID_DATA:
	case ESDFS_TREE_ANDROID_OBB:
	case ESDFS_TREE_ANDROID_MEDIA:
		if (ESDFS_RESTRICT_PERMS(sbi))
			inode->i_mode |= 0771;
		break;

	case ESDFS_TREE_ANDROID_APP:
		if (inode_i->appid)
			i_uid_write(inode, derive_uid(inode_i, inode_i->appid));
		if (ESDFS_RESTRICT_PERMS(sbi))
			inode->i_mode |= 0770;
		break;

	case ESDFS_TREE_ANDROID_USER:
		if (ESDFS_RESTRICT_PERMS(sbi)) {
			i_gid_write(inode, AID_SDCARD_ALL);
			inode->i_mode |= 0770;
		}
		inode->i_mode |= 0770;
		break;
	}

	/* strip execute bits from any non-directories */
	if (!S_ISDIR(inode->i_mode))
		inode->i_mode &= ~S_IXUGO;
}

/*
 * Before rerouting a lookup to follow a pseudo hard link, make sure that
 * a stub exists at the source.  Without it, readdir won't see an entry there
 * resulting in a strange user experience.
 */
static int lookup_link_source(struct dentry *dentry, struct dentry *parent)
{
	struct path lower_parent_path, lower_path;
	int err;

	esdfs_get_lower_path(parent, &lower_parent_path);

	/* Check if the stub user profile obb is there. */
	err = esdfs_lookup_nocase(&lower_parent_path, &dentry->d_name,
					&lower_path);
	/* Remember it to handle renames and removal. */
	if (!err)
		esdfs_set_lower_stub_path(dentry, &lower_path);

	esdfs_put_lower_path(parent, &lower_parent_path);

	return err;
}

int esdfs_derived_lookup(struct dentry *dentry, struct dentry **parent)
{
	struct esdfs_sb_info *sbi = ESDFS_SB((*parent)->d_sb);
	struct esdfs_inode_info *parent_i = ESDFS_I((*parent)->d_inode);

	/* Deny access to security-sensitive entries. */
	if (ESDFS_I((*parent)->d_inode)->tree == ESDFS_TREE_ROOT &&
	    match_name(&dentry->d_name, names_secure)) {
		pr_debug("esdfs: denying access to: %s", dentry->d_name.name);
		return -EACCES;
	}

	/* Pin the unified mode obb link parent as it flies by. */
	if (!sbi->obb_parent &&
	    test_opt(sbi, DERIVE_UNIFIED) &&
	    parent_i->tree == ESDFS_TREE_ROOT &&
	    parent_i->userid == 0 &&
	    !strncasecmp(dentry->d_name.name, "Android", dentry->d_name.len))
		sbi->obb_parent = dget(dentry);		/* keep it pinned */

	/*
	 * Handle obb directory "grafting" as a pseudo hard link by overriding
	 * its parent to point to the target obb directory's parent.  The rest
	 * of the lookup process will take care of setting up the bottom half
	 * to point to the real obb directory.
	 */
	if (parent_i->tree == ESDFS_TREE_ANDROID &&
	    ESDFS_DENTRY_NEEDS_LINK(dentry) &&
	    lookup_link_source(dentry, *parent) == 0) {
		BUG_ON(!sbi->obb_parent);
		if (ESDFS_INODE_CAN_LINK((*parent)->d_inode))
			*parent = dget(sbi->obb_parent);
	}
	return 0;
}

int esdfs_derived_revalidate(struct dentry *dentry, struct dentry *parent)
{
	/*
	 * If obb is not linked yet, it means the dentry is pointing to the
	 * stub.  Invalidate the dentry to force another lookup.
	 */
	if (ESDFS_I(parent->d_inode)->tree == ESDFS_TREE_ANDROID &&
	    ESDFS_INODE_CAN_LINK(dentry->d_inode) &&
	    ESDFS_DENTRY_NEEDS_LINK(dentry) &&
	    !ESDFS_DENTRY_IS_LINKED(dentry))
		return -ESTALE;

	return 0;
}

/*
 * Implement the extra checking that is done based on the caller's package
 * list-based access rights.
 */
int esdfs_check_derived_permission(struct inode *inode, int mask)
{
	const struct cred *cred;
	uid_t uid, appid;

	/*
	 * If we don't need to restrict access based on app GIDs and confine
	 * writes to outside of the Android/... tree, we can skip all of this.
	 */
	if (!ESDFS_RESTRICT_PERMS(ESDFS_SB(inode->i_sb)) &&
	    !test_opt(ESDFS_SB(inode->i_sb), DERIVE_CONFINE))
			return 0;

	cred = current_cred();
	uid = from_kuid(&init_user_ns, cred->uid);
	appid = uid % PKG_APPID_PER_USER;

	/* Reads, owners, and root are always granted access */
	if (!(mask & (MAY_WRITE | ESDFS_MAY_CREATE)) ||
	    uid == 0 || uid_eq(cred->uid, inode->i_uid))
		return 0;

	/*
	 * Grant access to sdcard_rw holders, unless we are in unified mode
	 * and we are trying to write to the protected /Android tree or to
	 * create files in the root (aka, "confined" access).
	 */
	if ((!test_opt(ESDFS_SB(inode->i_sb), DERIVE_UNIFIED) ||
	     (ESDFS_I(inode)->tree != ESDFS_TREE_ANDROID &&
	      ESDFS_I(inode)->tree != ESDFS_TREE_ANDROID_DATA &&
	      ESDFS_I(inode)->tree != ESDFS_TREE_ANDROID_OBB &&
	      ESDFS_I(inode)->tree != ESDFS_TREE_ANDROID_MEDIA &&
	      ESDFS_I(inode)->tree != ESDFS_TREE_ANDROID_APP &&
	      (ESDFS_I(inode)->tree != ESDFS_TREE_ROOT ||
	       !(mask & ESDFS_MAY_CREATE)))))
		return 0;

	pr_debug("esdfs: %s: denying access to appid: %u\n", __func__, appid);
	return -EACCES;
}

/*
 * The sdcard service has a hack that creates .nomedia files along certain
 * paths to stop MediaScanner.  Create those here.
 */
int esdfs_derive_mkdir_contents(struct dentry *dir_dentry)
{
	struct esdfs_inode_info *inode_i;
	struct qstr nomedia;
	struct dentry *lower_dentry;
	struct path lower_dir_path, lower_path;
	struct dentry *lower_parent_dentry = NULL;
	umode_t mode;
	int err = 0;

	if (!dir_dentry->d_inode)
		return 0;

	inode_i = ESDFS_I(dir_dentry->d_inode);

	/*
	 * Only create .nomedia in Android/data and Android/obb, but never in
	 * pseudo link stubs.
	 */
	if ((inode_i->tree != ESDFS_TREE_ANDROID_DATA &&
	     inode_i->tree != ESDFS_TREE_ANDROID_OBB) ||
	    (ESDFS_INODE_CAN_LINK(dir_dentry->d_inode) &&
	     ESDFS_DENTRY_NEEDS_LINK(dir_dentry) &&
	     !ESDFS_DENTRY_IS_LINKED(dir_dentry)))
		return 0;

	nomedia.name = ".nomedia";
	nomedia.len = strlen(nomedia.name);
	nomedia.hash = full_name_hash(NULL, nomedia.name, nomedia.len);

	esdfs_get_lower_path(dir_dentry, &lower_dir_path);

	/* check if lower has its own hash */
	if (lower_dir_path.dentry->d_flags & DCACHE_OP_HASH)
		lower_dir_path.dentry->d_op->d_hash(lower_dir_path.dentry,
							&nomedia);

	/* See if the lower file is there already. */
	err = vfs_path_lookup(lower_dir_path.dentry, lower_dir_path.mnt,
			      nomedia.name, 0, &lower_path);
	if (!err)
		path_put(&lower_path);
	/* If it's there or there was an error, we're done */
	if (!err || err != -ENOENT)
		goto out;

	/* The lower file is not there.  See if the dentry is in the cache. */
	lower_dentry = d_lookup(lower_dir_path.dentry, &nomedia);
	if (!lower_dentry) {
		/* It's not there, so create a negative lower dentry. */
		lower_dentry = d_alloc(lower_dir_path.dentry, &nomedia);
		if (!lower_dentry) {
			err = -ENOMEM;
			goto out;
		}
		d_add(lower_dentry, NULL);
	}

	/* Now create the lower file. */
	mode = S_IFREG;
	lower_parent_dentry = lock_parent(lower_dentry);
	esdfs_set_lower_mode(ESDFS_SB(dir_dentry->d_sb), &mode);
	err = vfs_create(lower_dir_path.dentry->d_inode, lower_dentry, mode,
			 true);
	unlock_dir(lower_parent_dentry);
	dput(lower_dentry);

out:
	esdfs_put_lower_path(dir_dentry, &lower_dir_path);

	return err;
}
