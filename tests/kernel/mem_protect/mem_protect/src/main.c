/*
 * Parts derived from tests/kernel/fatal/src/main.c, which has the
 * following copyright and license:
 *
 * Copyright (c) 2017, 2020 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <ztest.h>
#include <kernel_structs.h>
#include <string.h>
#include <stdlib.h>
#include "mem_protect.h"


void test_main(void)
{
	k_thread_priority_set(k_current_get(), -1);

	ztest_test_suite(memory_protection_test_suite,
			 ztest_unit_test(test_permission_inheritance),
			 ztest_unit_test(test_mem_domain_valid_access),
			 ztest_unit_test(test_mem_domain_invalid_access),
			 ztest_unit_test(test_mem_domain_partitions_user_rw),
			 ztest_unit_test(test_mem_domain_partitions_supervisor_rw),
			 ztest_unit_test(test_mem_domain_partitions_user_ro),
			 ztest_unit_test(test_mem_domain_add_partitions_invalid),
			 ztest_unit_test(test_mem_domain_add_partitions_simple),
			 ztest_unit_test(test_mem_domain_remove_partitions_simple),
			 ztest_unit_test(test_mem_domain_remove_partitions),
			 ztest_unit_test(test_mark_thread_exit_uninitialized),
			 ztest_unit_test(
				 test_mem_domain_api_kernel_thread_only),
			 ztest_user_unit_test(
				 test_mem_domain_api_kernel_thread_only),
			 ztest_unit_test(test_mem_part_auto_determ_size),
			 ztest_unit_test(
				 test_mem_part_auto_determ_size_per_mpu),
			 ztest_unit_test(test_mem_part_inherit_by_child_thr),
			 ztest_unit_test(test_macros_obtain_names_data_bss),
			 ztest_unit_test(test_mem_part_assign_bss_vars_zero),
			 ztest_unit_test(test_kobject_access_grant),
			 ztest_unit_test(test_syscall_invalid_kobject),
			 ztest_unit_test(test_thread_without_kobject_permission),
			 ztest_unit_test(test_kobject_revoke_access),
			 ztest_unit_test(test_kobject_grant_access_kobj),
			 ztest_unit_test(test_kobject_grant_access_kobj_invalid),
			 ztest_unit_test(test_kobject_release_from_user),
			 ztest_unit_test(test_kobject_access_all_grant),
			 ztest_unit_test(test_thread_has_residual_permissions),
			 ztest_unit_test(test_kobject_access_grant_to_invalid_thread),
			 ztest_user_unit_test(test_kobject_access_invalid_kobject),
			 ztest_user_unit_test(test_access_kobject_without_init_access),
			 ztest_unit_test(test_access_kobject_without_init_with_access),
			 ztest_unit_test(test_kobject_reinitialize_thread_kobj),
			 ztest_unit_test(test_create_new_thread_from_user),
			 ztest_unit_test(test_mark_thread_exit_uninitialized),
			 ztest_unit_test(
				 test_new_user_thread_with_in_use_stack_obj),
			 ztest_unit_test(
				 test_create_new_thread_from_user_no_access_stack),
			 ztest_unit_test(
				 test_create_new_thread_from_user_invalid_stacksize),
			 ztest_unit_test(
				 test_create_new_thread_from_user_huge_stacksize),
			 ztest_unit_test(test_create_new_supervisor_thread_from_user),
			 ztest_unit_test(test_create_new_essential_thread_from_user),
			 ztest_unit_test(test_create_new_higher_prio_thread_from_user),
			 ztest_unit_test(test_inherit_resource_pool),
			 ztest_unit_test(test_create_new_invalid_prio_thread_from_user)
			 );
	ztest_run_test_suite(memory_protection_test_suite);
}
