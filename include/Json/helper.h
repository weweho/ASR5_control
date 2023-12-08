#ifndef CBOR_HELPER_H
#define CBOR_HELPER_H

#if defined(__cplusplus)
extern "C" {
#endif

#include "cbor.h"

const char *cbor_stringify_error(cbor_error_t err);
const char *cbor_stringify_item(cbor_item_t *item);

#if defined(__cplusplus)
}
#endif

#endif /* CBOR_HELPER_H */
