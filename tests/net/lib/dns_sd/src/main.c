/*
 * Copyright (c) 2020 Friedt Professional Engineering Services, Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <stdint.h>

#include <ztest.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(test_net_dns_sd, LOG_LEVEL_DBG);

#include "dns_pack.h"
#include "dns_sd.h"

#define BUFSZ 256
#define IP_ADDR(a, b, c, d) ((uint32_t)		  \
			     0			  \
			     | ((a & 0xff) << 24) \
			     | ((b & 0xff) << 16) \
			     | ((c & 0xff) <<  8) \
			     | ((d & 0xff) <<  0) \
			     )

/**
 * Variant of @ref DNS_SD_REGISTER_SERVICE only used for testing
 *
 * This variant sidesteps compile-time checks and allows the
 * caller to specify a domain other than "local".
 */
#define TEST_DNS_SD_REGISTER_SERVICE(id, instance, service,	   \
				     proto, domain, text, port)	   \
	static const Z_STRUCT_SECTION_ITERABLE(dns_sd_rec, id) = { \
		instance,					   \
		service,					   \
		proto,						   \
		domain,						   \
		text,						   \
		sizeof(text) - 1,				   \
		port,						   \
	}

extern bool label_is_valid(const char *label, size_t label_size);
extern int add_a_record(const struct dns_sd_rec *inst, uint32_t ttl,
			uint16_t host_offset, uint32_t addr,
			uint8_t *buf,
			uint16_t buf_offset, uint16_t buf_size);
extern int add_ptr_record(const struct dns_sd_rec *inst, uint32_t ttl,
			  uint8_t *buf, uint16_t buf_offset,
			  uint16_t buf_size,
			  uint16_t *service_offset,
			  uint16_t *instance_offset,
			  uint16_t *domain_offset);
extern int add_txt_record(const struct dns_sd_rec *inst, uint32_t ttl,
			  uint16_t instance_offset, uint8_t *buf,
			  uint16_t buf_offset, uint16_t buf_size);
extern int add_aaaa_record(const struct dns_sd_rec *inst, uint32_t ttl,
			   uint16_t host_offset, const uint8_t addr[16],
			   uint8_t *buf, uint16_t buf_offset,
			   uint16_t buf_size);
extern int add_srv_record(const struct dns_sd_rec *inst, uint32_t ttl,
			  uint16_t instance_offset,
			  uint16_t domain_offset,
			  uint8_t *buf, uint16_t buf_offset,
			  uint16_t buf_size,
			  uint16_t *host_offset);
extern size_t dns_sd_service_proto_size(const struct dns_sd_rec *ref);
extern bool dns_sd_rec_is_valid(const struct dns_sd_rec *ref);

static const uint8_t nasxxxxxx_text[] = "\x06path=/";
DNS_SD_REGISTER_TCP_SERVICE(nasxxxxxx, "NASXXXXXX", "_http",
			    nasxxxxxx_text, 8080);

static uint8_t create_query_buf[BUFSZ];
static uint8_t *create_query(const struct dns_sd_rec *inst,
			     uint8_t rr_type, size_t *size)
{
	uint16_t offs = 0;
	uint8_t label_size;
	uint16_t service_proto_size = dns_sd_service_proto_size(inst);

	uint16_t expected_req_buf_size = 0
					 + sizeof(struct dns_header)
					 + service_proto_size
					 + sizeof(struct dns_query);

	struct dns_header *hdr =
		(struct dns_header *)&create_query_buf[0];

	hdr->id = htons(0);
	hdr->qdcount = htons(1);
	offs += sizeof(struct dns_header);

	label_size = strlen(inst->service);
	create_query_buf[offs++] = label_size;
	memcpy(&create_query_buf[offs], inst->service, label_size);
	offs += label_size;

	label_size = strlen(inst->proto);
	create_query_buf[offs++] = label_size;
	memcpy(&create_query_buf[offs], inst->proto, label_size);
	offs += label_size;

	label_size = strlen(inst->domain);
	create_query_buf[offs++] = label_size;
	memcpy(&create_query_buf[offs], inst->domain, label_size);
	offs += label_size;

	create_query_buf[offs++] = '\0';

	struct dns_query *query =
		(struct dns_query *)&create_query_buf[offs];
	query->type = htons(rr_type);
	query->class_ = htons(DNS_CLASS_IN);
	offs += sizeof(struct dns_query);

	zassert_equal(expected_req_buf_size, offs,
		      "sz: %zu offs: %u", expected_req_buf_size, offs);

	*size = offs;

	return create_query_buf;
}

static void test_label_is_valid(void)
{
	zassert_equal(false, label_is_valid(NULL,
					    DNS_LABEL_MIN_SIZE), "");
	zassert_equal(false, label_is_valid("",
					    DNS_LABEL_MIN_SIZE - 1),
		      "");
	zassert_equal(false, label_is_valid(
			      "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
			      "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
			      DNS_LABEL_MAX_SIZE + 1), "");
	zassert_equal(true,  label_is_valid("a",
					    DNS_LABEL_MIN_SIZE), "");
	zassert_equal(true,  label_is_valid(
			      "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
			      "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
			      DNS_LABEL_MAX_SIZE), "");
	zassert_equal(false, label_is_valid("9abc", 4), "");
	zassert_equal(true,  label_is_valid("a9bc", 4), "");
	zassert_equal(false, label_is_valid("-abc", 4), "");
	zassert_equal(true,  label_is_valid("a-bc", 4), "");
	zassert_equal(true,  label_is_valid("A-Bc", 4), "");
}

static void test_dns_sd_rec_is_valid(void)
{
	TEST_DNS_SD_REGISTER_SERVICE(name_min,
				     "x",
				     "_x",
				     "_tcp",
				     "xx",
				     DNS_SD_EMPTY_TXT,
				     8080);
	zassert_equal(true, dns_sd_rec_is_valid(&name_min), "");

	TEST_DNS_SD_REGISTER_SERVICE(name_max,
				     "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
				     "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx",
				     "_xxxxxxxxxxxxxxx",
				     "_tcp",
				     "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
				     "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx",
				     DNS_SD_EMPTY_TXT,
				     8080);
	zassert_equal(true, dns_sd_rec_is_valid(&name_max), "");

	TEST_DNS_SD_REGISTER_SERVICE(label_too_small,
				     "x",
				     "_",
				     "_tcp",
				     "xx",
				     DNS_SD_EMPTY_TXT,
				     8080);
	zassert_equal(false, dns_sd_rec_is_valid(&label_too_small), "");

	TEST_DNS_SD_REGISTER_SERVICE(label_too_big,
				     "x",
				     "_x",
				     "_tcp",
				     "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
				     "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx",
				     DNS_SD_EMPTY_TXT,
				     8080);
	zassert_equal(false, dns_sd_rec_is_valid(&label_too_big), "");

	TEST_DNS_SD_REGISTER_SERVICE(invalid_instance,
				     "abc" "\x01" "def",
				     "_x",
				     "_tcp",
				     "xx",
				     DNS_SD_EMPTY_TXT,
				     8080);
	zassert_equal(false, dns_sd_rec_is_valid(&invalid_instance), "");

	TEST_DNS_SD_REGISTER_SERVICE(invalid_service_prefix,
				     "x",
				     "xx",
				     "_tcp",
				     "xx",
				     DNS_SD_EMPTY_TXT,
				     8080);
	zassert_equal(false, dns_sd_rec_is_valid(
			      &invalid_service_prefix), "");

	TEST_DNS_SD_REGISTER_SERVICE(invalid_service,
				     "x",
				     "_x.y",
				     "_tcp",
				     "xx",
				     DNS_SD_EMPTY_TXT,
				     8080);
	zassert_equal(false, dns_sd_rec_is_valid(&invalid_service), "");

	TEST_DNS_SD_REGISTER_SERVICE(invalid_proto,
				     "x",
				     "_y",
				     "_wtf",
				     "xx",
				     DNS_SD_EMPTY_TXT,
				     8080);
	zassert_equal(false, dns_sd_rec_is_valid(&invalid_proto), "");

	/* We do not currently support subdomains */
	TEST_DNS_SD_REGISTER_SERVICE(invalid_domain,
				     "x",
				     "_x",
				     "_tcp",
				     "x.y",
				     DNS_SD_EMPTY_TXT,
				     8080);
	zassert_equal(false, dns_sd_rec_is_valid(&invalid_domain), "");

	zassert_equal(true, dns_sd_rec_is_valid(&nasxxxxxx), "");
}

static void test_create_query(void)
{
	size_t actual_query_size = -1;
	uint8_t *actual_query = create_query(&nasxxxxxx,
					     DNS_RR_TYPE_PTR,
					     &actual_query_size);
	uint8_t expected_query[] = {
		0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x05, 0x5f, 0x68, 0x74,
		0x74, 0x70, 0x04, 0x5f, 0x74, 0x63, 0x70, 0x05,
		0x6c, 0x6f, 0x63, 0x61, 0x6c, 0x00, 0x00, 0x0c,
		0x00, 0x01
	};
	size_t expected_query_size = sizeof(expected_query);

	zassert_equal(actual_query_size, expected_query_size, "");
	zassert_mem_equal(expected_query, actual_query,
			  MIN(actual_query_size, expected_query_size),
			  "");
}

static void test_add_ptr_record(void)
{
	const uint32_t ttl = DNS_SD_PTR_TTL;
	const uint32_t offset = sizeof(struct dns_header);

	uint16_t service_offset = -1;
	uint16_t instance_offset = -1;
	uint16_t domain_offset = -1;

	static uint8_t actual_buf[BUFSZ];
	static const uint8_t expected_buf[] = {
		0x05, 0x5f, 0x68, 0x74, 0x74, 0x70, 0x04, 0x5f,
		0x74, 0x63, 0x70, 0x05, 0x6c, 0x6f, 0x63, 0x61,
		0x6c, 0x00, 0x00, 0x0c, 0x00, 0x01, 0x00, 0x00,
		0x11, 0x94, 0x00, 0x0c, 0x09, 0x4e, 0x41, 0x53,
		0x58, 0x58, 0x58, 0x58, 0x58, 0x58, 0xc0, 0x0c,
	};
	int expected_int = sizeof(expected_buf);

	int actual_int = add_ptr_record(&nasxxxxxx, ttl,
					actual_buf, offset,
					sizeof(actual_buf),
					&service_offset,
					&instance_offset,
					&domain_offset);

	zassert_equal(actual_int, expected_int, "");

	zassert_equal(instance_offset, 40, "");
	zassert_equal(domain_offset, 23, "");

	memmove(actual_buf, actual_buf + offset, actual_int);
	zassert_mem_equal(actual_buf, expected_buf,
			  MIN(actual_int, expected_int), "");

	/* dns_sd_rec_is_valid failure */
	TEST_DNS_SD_REGISTER_SERVICE(null_label,
				     NULL,
				     "_x",
				     "_tcp",
				     "xx",
				     DNS_SD_EMPTY_TXT,
				     8080);
	zassert_equal(-EINVAL, add_ptr_record(&null_label, ttl,
					      actual_buf, offset,
					      actual_int,
					      &service_offset,
					      &instance_offset,
					      &domain_offset), "");

	/* buffer too small failure */
	zassert_equal(-ENOSPC, add_ptr_record(&nasxxxxxx, ttl,
					      actual_buf, offset, 0,
					      &service_offset,
					      &instance_offset,
					      &domain_offset), "");

	/* offset too big for message compression (service) */
	zassert_equal(-E2BIG, add_ptr_record(&nasxxxxxx, ttl,
					     actual_buf, DNS_SD_PTR_MASK,
					     0xffff, &service_offset,
					     &instance_offset,
					     &domain_offset), "");

	/* offset too big for message compression (instance) */
	zassert_equal(-E2BIG, add_ptr_record(&nasxxxxxx, ttl,
					     actual_buf, 0x3fff,
					     0xffff, &service_offset,
					     &instance_offset,
					     &domain_offset), "");
}

static void test_add_txt_record(void)
{
	const uint32_t ttl = DNS_SD_TXT_TTL;
	const uint32_t offset = 0;
	const uint16_t instance_offset = 0x28;

	static uint8_t actual_buf[BUFSZ];
	static const uint8_t expected_buf[] = {
		0xc0, 0x28, 0x00, 0x10, 0x80, 0x01, 0x00, 0x00,
		0x11, 0x94, 0x00, 0x07, 0x06, 0x70, 0x61, 0x74,
		0x68, 0x3d, 0x2f
	};
	int expected_int = sizeof(expected_buf);

	int actual_int = add_txt_record(&nasxxxxxx, ttl,
					instance_offset, actual_buf,
					offset,
					sizeof(actual_buf));

	zassert_equal(actual_int, expected_int, "");

	zassert_mem_equal(actual_buf, expected_buf, MIN(actual_int,
							expected_int),
			  "");

	/* too big for message compression */
	zassert_equal(-E2BIG, add_txt_record(&nasxxxxxx, ttl, DNS_SD_PTR_MASK,
					     actual_buf, offset,
					     sizeof(actual_buf)), "");

	/* buffer too small */
	zassert_equal(-ENOSPC, add_txt_record(&nasxxxxxx, ttl, offset,
					      actual_buf, offset,
					      0), "");
}

static void test_add_srv_record(void)
{
	const uint32_t ttl = DNS_SD_SRV_TTL;
	const uint32_t offset = 0;
	const uint16_t instance_offset = 0x28;
	const uint16_t domain_offset = 0x17;

	uint16_t host_offset = -1;
	static uint8_t actual_buf[BUFSZ];
	static const uint8_t expected_buf[] = {
		0xc0, 0x28, 0x00, 0x21, 0x80, 0x01, 0x00, 0x00,
		0x00, 0x78, 0x00, 0x12, 0x00, 0x00, 0x00, 0x00,
		0x1f, 0x90, 0x09, 0x4e, 0x41, 0x53, 0x58, 0x58,
		0x58, 0x58, 0x58, 0x58, 0xc0, 0x17
	};

	int expected_int = sizeof(expected_buf);
	int actual_int = add_srv_record(&nasxxxxxx, ttl,
					instance_offset, domain_offset,
					actual_buf,
					offset, sizeof(actual_buf),
					&host_offset);

	zassert_equal(actual_int, expected_int, "");

	zassert_equal(host_offset, 18, "");

	zassert_mem_equal(actual_buf, expected_buf,
			  MIN(actual_int, expected_int), "");

	/* offset too big for message compression (instance) */
	zassert_equal(-E2BIG, add_srv_record(&nasxxxxxx, ttl, DNS_SD_PTR_MASK,
					     domain_offset,
					     actual_buf, offset,
					     sizeof(actual_buf),
					     &host_offset), "");

	/* offset too big for message compression (domain) */
	zassert_equal(-E2BIG, add_srv_record(&nasxxxxxx, ttl,
					     instance_offset, DNS_SD_PTR_MASK,
					     actual_buf, offset,
					     sizeof(actual_buf),
					     &host_offset), "");

	/* buffer too small */
	zassert_equal(-ENOSPC, add_srv_record(&nasxxxxxx, ttl,
					      instance_offset,
					      domain_offset,
					      actual_buf,
					      offset, 0,
					      &host_offset), "");
}

static void test_add_a_record(void)
{
	const uint32_t ttl = DNS_SD_A_TTL;
	const uint32_t offset = 0;
	const uint16_t host_offset = 0x59;
	/* this one is made up */
	const uint32_t addr = IP_ADDR(177, 5, 240, 13);

	static uint8_t actual_buf[BUFSZ];
	static const uint8_t expected_buf[] = {
		0xc0, 0x59, 0x00, 0x01, 0x80, 0x01, 0x00, 0x00,
		0x00, 0x78, 0x00, 0x04, 0xb1, 0x05, 0xf0, 0x0d,
	};

	int expected_int = sizeof(expected_buf);
	int actual_int = add_a_record(&nasxxxxxx, ttl, host_offset,
				      addr, actual_buf, offset,
				      sizeof(actual_buf));

	zassert_equal(actual_int, expected_int, "");

	zassert_mem_equal(actual_buf, expected_buf,
			  MIN(actual_int, expected_int), "");

	/* test offset too large */
	zassert_equal(-E2BIG, add_a_record(&nasxxxxxx, ttl, DNS_SD_PTR_MASK,
					   addr, actual_buf, offset,
					   sizeof(actual_buf)), "");

	/* test buffer too small */
	zassert_equal(-ENOSPC, add_a_record(&nasxxxxxx, ttl,
					    host_offset, addr,
					    actual_buf, offset,
					    0), "");
}

static void test_add_aaaa_record(void)
{
	const uint32_t ttl = DNS_SD_AAAA_TTL;
	const uint32_t offset = 0;
	const uint16_t host_offset = 0x59;
	/* this one is made up */
	const uint8_t addr[16] = {
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1
	};

	static uint8_t actual_buf[BUFSZ];
	static const uint8_t expected_buf[] = {
		0xc0, 0x59, 0x00, 0x1c, 0x80, 0x01, 0x00, 0x00,
		0x00, 0x78, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x01,
	};

	int expected_int = sizeof(expected_buf);
	int actual_int = add_aaaa_record(&nasxxxxxx, ttl, host_offset,
					 addr, actual_buf, offset,
					 sizeof(actual_buf));

	zassert_equal(actual_int, expected_int, "");

	zassert_mem_equal(actual_buf, expected_buf,
			  MIN(actual_int, expected_int), "");

	/* offset too large for message compression */
	zassert_equal(-E2BIG, add_aaaa_record(&nasxxxxxx, ttl, DNS_SD_PTR_MASK,
					      addr, actual_buf,
					      offset,
					      sizeof(actual_buf)), "");

	/* buffer too small */
	zassert_equal(-ENOSPC,
		      add_aaaa_record(&nasxxxxxx, ttl, host_offset,
				      addr, actual_buf,
				      offset, 0), "");
}

static void test_dns_sd_handle_ptr_query(void)
{
	const uint32_t addr = IP_ADDR(177, 5, 240, 13);
	uint8_t actual_rsp[512] = {};
	size_t req_size;
	uint8_t *req = create_query(&nasxxxxxx, DNS_RR_TYPE_PTR,
				    &req_size);

	zassert_not_equal(req, NULL, "");

	int actual_int = dns_sd_handle_ptr_query(&nasxxxxxx, false,
						 (uint8_t *)&addr,
						 (struct dns_query *)req,
						 &actual_rsp[0],
						 sizeof(actual_rsp) -
						 sizeof(struct dns_header));

	zassert_true(actual_int > 0,
		     "dns_sd_handle_ptr_query() failed (%d)",
		     actual_int);

	uint8_t expected_rsp[] = {
		0x00, 0x00, 0x84, 0x00, 0x00, 0x00, 0x00, 0x01,
		0x00, 0x00, 0x00, 0x03, 0x05, 0x5f, 0x68, 0x74,
		0x74, 0x70, 0x04, 0x5f, 0x74, 0x63, 0x70, 0x05,
		0x6c, 0x6f, 0x63, 0x61, 0x6c, 0x00, 0x00, 0x0c,
		0x00, 0x01, 0x00, 0x00, 0x11, 0x94, 0x00, 0x0c,
		0x09, 0x4e, 0x41, 0x53, 0x58, 0x58, 0x58, 0x58,
		0x58, 0x58, 0xc0, 0x0c, 0xc0, 0x28, 0x00, 0x10,
		0x80, 0x01, 0x00, 0x00, 0x11, 0x94, 0x00, 0x07,
		0x06, 0x70, 0x61, 0x74, 0x68, 0x3d, 0x2f, 0xc0,
		0x28, 0x00, 0x21, 0x80, 0x01, 0x00, 0x00, 0x00,
		0x78, 0x00, 0x12, 0x00, 0x00, 0x00, 0x00, 0x1f,
		0x90, 0x09, 0x4e, 0x41, 0x53, 0x58, 0x58, 0x58,
		0x58, 0x58, 0x58, 0xc0, 0x17, 0xc0, 0x59, 0x00,
		0x01, 0x80, 0x01, 0x00, 0x00, 0x00, 0x78, 0x00,
		0x04, 0xb1, 0x05, 0xf0, 0x0d,
	};

	int expected_int = sizeof(expected_rsp);

	zassert_equal(actual_int, expected_int, "");

	zassert_mem_equal(actual_rsp, expected_rsp,
			  MIN(actual_int, expected_int), "");
}

void test_main(void)
{
	ztest_test_suite(dns_sd_tests,
			 ztest_unit_test(test_label_is_valid),
			 ztest_unit_test(test_dns_sd_rec_is_valid),
			 ztest_unit_test(test_create_query),
			 ztest_unit_test(test_add_ptr_record),
			 ztest_unit_test(test_add_txt_record),
			 ztest_unit_test(test_add_srv_record),
			 ztest_unit_test(test_add_a_record),
			 ztest_unit_test(test_add_aaaa_record),
			 ztest_unit_test(test_dns_sd_handle_ptr_query));

	ztest_run_test_suite(dns_sd_tests);
}
