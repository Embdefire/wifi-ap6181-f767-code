#if defined(WICED)
#include "wiced_duktape_tests_api_internal.h"
#endif

/*
 *  Test for duk_(p)eval_string() automatic .fileName change in Duktape 1.5.x:
 *  https://github.com/svaarala/duktape/issues/516
 */

/*===
*** test_1 (duk_safe_call)
rc: 1
err.fileName: eval
err.lineNumber: 3
final top: 1
==> rc=0, result='undefined'
===*/

static duk_ret_t test_1(duk_context *ctx, void *udata) {
	duk_int_t rc;

	(void) udata;

	rc = duk_peval_string(ctx, "\n\naiee");
	printf("rc: %ld\n", (long) rc);

	duk_get_prop_string(ctx, -1, "fileName");
	printf("err.fileName: %s\n", duk_require_string(ctx, -1));
	duk_pop(ctx);

	duk_get_prop_string(ctx, -1, "lineNumber");
	printf("err.lineNumber: %ld\n", (long) duk_require_int(ctx, -1));
	duk_pop(ctx);

	printf("final top: %ld\n", (long) duk_get_top(ctx));
	return 0;
}

#if defined(WICED)
void wiced_duktape_tests_api_eval_filename(duk_context *ctx) {
#else
void test(duk_context *ctx) {
#endif
	TEST_SAFE_CALL(test_1);
}
