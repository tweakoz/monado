// Copyright 2018-2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Unit tests.
 * @author Christoph Haag <christop.haag@collabora.com>
 */

#include "tests_common.h"

#include "xrt/state_trackers/oxr/oxr_logger.h"
// include c files to test static functions

#include "xrt/state_trackers/oxr/oxr_space.c"

// system test will validate API calls to set up state so we don't do it here'

struct {
  XrInstance instance;
  XrSystemId system_id;
  XrSession session;

  XrGraphicsBindingOpenGLXlibKHR *graphics_binding_gl;
  uint32_t view_count;
  XrViewConfigurationView *configuration_views;
} oxr;

void reset_struct(void) {
  oxr.instance = XR_NULL_HANDLE;
  oxr.system_id = XR_NULL_SYSTEM_ID;
  oxr.session = XR_NULL_HANDLE;
  oxr.graphics_binding_gl = NULL;
  oxr.view_count = 0;
  oxr.configuration_views = NULL;
}

void setup_opengl(void) {
  reset_struct();

  const char *extensions[] = {XR_KHR_OPENGL_ENABLE_EXTENSION_NAME};

  XrInstanceCreateInfo ici = {
      .type = XR_TYPE_INSTANCE_CREATE_INFO,
      .enabledExtensionCount = sizeof(extensions) / sizeof(extensions[0]),
      .enabledExtensionNames = extensions,
  };
  oxr_xrCreateInstance(&ici, &oxr.instance);

  XrSystemGetInfo sgi = {.type = XR_TYPE_SYSTEM_GET_INFO,
                         .formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY};
  oxr_xrGetSystem(oxr.instance, &sgi, &oxr.system_id);

  oxr.graphics_binding_gl = malloc(sizeof(XrGraphicsBindingOpenGLXlibKHR));
  oxr.graphics_binding_gl->next = NULL;
  oxr.graphics_binding_gl->type = XR_TYPE_GRAPHICS_BINDING_OPENGL_XLIB_KHR;

  init_glx(&oxr.graphics_binding_gl->xDisplay,
           &oxr.graphics_binding_gl->glxDrawable,
           &oxr.graphics_binding_gl->glxContext);

  XrSessionCreateInfo sci = {.type = XR_TYPE_SESSION_CREATE_INFO,
                             .next = oxr.graphics_binding_gl,
                             .systemId = oxr.system_id};

  XrSession session;
  oxr_xrCreateSession(oxr.instance, &sci, &session);

  XrViewConfigurationType configuration =
      XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
  oxr_xrEnumerateViewConfigurationViews(
      oxr.instance, oxr.system_id, configuration, 0, &oxr.view_count, NULL);

  oxr.configuration_views =
      malloc(sizeof(XrViewConfigurationView) * oxr.view_count);

  oxr_xrEnumerateViewConfigurationViews(
      oxr.instance, oxr.system_id, configuration, oxr.view_count,
      &oxr.view_count, oxr.configuration_views);
}

void setup_headless(void) {
  reset_struct();

  const char *extensions[] = {XR_KHR_HEADLESS_EXTENSION_NAME};

  XrInstanceCreateInfo ici = {.type = XR_TYPE_INSTANCE_CREATE_INFO,
                              .enabledExtensionCount =
                                  sizeof(extensions) / sizeof(extensions[0]),
                              .enabledExtensionNames = extensions};
  oxr_xrCreateInstance(&ici, &oxr.instance);

  XrSystemGetInfo sgi = {.type = XR_TYPE_SYSTEM_GET_INFO,
                         .formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY};
  oxr_xrGetSystem(oxr.instance, &sgi, &oxr.system_id);

  XrSessionCreateInfo sci = {.type = XR_TYPE_SESSION_CREATE_INFO,
                             .next = NULL,
                             .systemId = oxr.system_id};

  XrSession session;
  oxr_xrCreateSession(oxr.instance, &sci, &oxr.session);
}

void teardown(void) {
  if (oxr.session != XR_NULL_HANDLE)
    oxr_xrDestroySession(oxr.session);
  if (oxr.instance != XR_NULL_HANDLE)
    oxr_xrDestroyInstance(oxr.instance);
  if (oxr.graphics_binding_gl) {
    destroy_glx();
    free(oxr.graphics_binding_gl);
  }
  if (oxr.configuration_views)
    free(oxr.configuration_views);

  reset_struct();
}

START_TEST(headless_unit_test) {

  uint32_t format_count = 0;
  ck_assert(oxr_xrEnumerateSwapchainFormats(oxr.session, 0, &format_count,
                                            NULL) == XR_SUCCESS);
  ck_assert_int_eq(format_count, 0);

  /// @todo test for validation error?
  /*
  XrSwapchainCreateInfo sci = { 0 };
  sci.type = XR_TYPE_SWAPCHAIN_CREATE_INFO;
  sci.next = NULL;
  XrSwapchain swapchain[1];
  REQUIRE(oxr_xrCreateSwapchain(session, &sci, swapchain) != XR_SUCCESS);
  */
}
END_TEST

START_TEST(space_unit_test) {
  XrPosef pose = {0};
  pose.orientation.w = 1;

  struct oxr_logger log;
  oxr_log_init(&log, "Test Logger");
  ck_assert(XR_SUCCESS ==
            check_reference_space_type(&log, XR_REFERENCE_SPACE_TYPE_LOCAL));
  ck_assert(XR_SUCCESS ==
            check_reference_space_type(&log, XR_REFERENCE_SPACE_TYPE_VIEW));

  XrReferenceSpaceCreateInfo rsci = {
      .type = XR_TYPE_REFERENCE_SPACE_CREATE_INFO,
      .referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL,
      .poseInReferenceSpace = pose};

  XrSpace space;
  oxr_xrCreateReferenceSpace(oxr.session, &rsci, &space);

  ck_assert_str_eq("local",
                   get_ref_space_type_short_str((struct oxr_space *)space));
  /// @todo
}
END_TEST

START_TEST(opengl_unit_test) {
  ck_assert_int_eq(oxr.view_count, 2);

  ck_assert_int_gt(oxr.configuration_views[0].recommendedImageRectWidth, 0);
  ck_assert_int_gt(oxr.configuration_views[0].recommendedImageRectHeight, 0);
  ck_assert_int_gt(oxr.configuration_views[0].maxImageRectWidth, 0);
  ck_assert_int_gt(oxr.configuration_views[0].maxImageRectHeight, 0);
  ck_assert_int_gt(oxr.configuration_views[0].recommendedSwapchainSampleCount,
                   0);
  ck_assert_int_gt(oxr.configuration_views[0].maxSwapchainSampleCount, 0);

  XrGraphicsRequirementsOpenGLKHR opengl_reqs = {
      .type = XR_TYPE_GRAPHICS_REQUIREMENTS_OPENGL_KHR, .next = NULL};
  oxr_xrGetOpenGLGraphicsRequirementsKHR(oxr.instance, oxr.system_id,
                                         &opengl_reqs);

  ck_assert_int_gt(XR_VERSION_MAJOR(opengl_reqs.minApiVersionSupported), 0);
  ck_assert_int_gt(XR_VERSION_MINOR(opengl_reqs.minApiVersionSupported), 0);
  ck_assert_int_ge(XR_VERSION_PATCH(opengl_reqs.minApiVersionSupported), 0);

  ck_assert_int_gt(XR_VERSION_MAJOR(opengl_reqs.maxApiVersionSupported), 0);
  ck_assert_int_gt(XR_VERSION_MINOR(opengl_reqs.maxApiVersionSupported), 0);
  ck_assert_int_ge(XR_VERSION_PATCH(opengl_reqs.maxApiVersionSupported), 0);

  /// @todo
}
END_TEST

Suite *unit_test_suite(void) {
  Suite *s = suite_create("Monado Unit Test Suite");
  TCase *headless_unit_tests = tcase_create("Headless Unit Tests");
  TCase *opengl_unit_tests = tcase_create("OpenGL Unit Tests");

  tcase_add_checked_fixture(headless_unit_tests, setup_headless, teardown);
  tcase_add_checked_fixture(opengl_unit_tests, setup_opengl, teardown);

  tcase_add_test(headless_unit_tests, headless_unit_test);
  tcase_add_test(headless_unit_tests, space_unit_test);
  tcase_add_test(opengl_unit_tests, opengl_unit_test);

  suite_add_tcase(s, headless_unit_tests);
  suite_add_tcase(s, opengl_unit_tests);

  return s;
}