/*******************************************************************************
 *   @file   st)assert.h
 *   @brief  definition of compile time assert.
 *   @author hylics
********************************************************************************/

#ifndef __CT_ASSERT_H__
#define __CT_ASSERT_H__

/* Note we need the 2 concats below because arguments to ##
 * are not expanded, so we need to expand __LINE__ with one indirection
 * before doing the actual concatenation. */
//#define ASSERT_CONCAT_(a, b) a##b
//#define ASSERT_CONCAT(a, b) ASSERT_CONCAT_(a, b)
///* These can't be used after statements in c89. */
//#ifdef __COUNTER__
//  #define STATIC_ASSERT(e,m) \
//    ;enum { ASSERT_CONCAT(static_assert_, __COUNTER__) = 1/(!!(e)) }
//#else
//  /* This can't be used twice on the same line so ensure if using in headers
//   * that the headers are not included twice (by wrapping in #ifndef...#endif)
//   * Note it doesn't cause an issue when used on same line of separate modules
//   * compiled with gcc -combine -fwhole-program.  */
//  #define STATIC_ASSERT(e,m) \
//    ;enum { ASSERT_CONCAT(assert_line_, __LINE__) = 1/(!!(e)) }
//#endif

//#define STATIC_ASSERT(e) {enum { ct_assert_value = 1/(!!(e)) };}

/* gnu */
#define STATIC_ASSERT(e) extern char (*ct_assert(void)) [sizeof(char[1 - 2*!(e)])]



#endif	// __CT_ASSERT_H__

