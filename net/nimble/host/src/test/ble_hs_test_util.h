/**
 * Copyright (c) 2015 Runtime Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef H_BLE_HS_TEST_UTIL_
#define H_BLE_HS_TEST_UTIL_

#include <inttypes.h>
struct ble_hs_conn;
struct ble_l2cap_chan;

extern struct os_mbuf *ble_hs_test_util_prev_tx;

void ble_hs_test_util_build_cmd_complete(uint8_t *dst, int len,
                                         uint8_t param_len, uint8_t num_pkts,
                                         uint16_t opcode);
void ble_hs_test_util_build_cmd_status(uint8_t *dst, int len,
                                       uint8_t status, uint8_t num_pkts,
                                       uint16_t opcode);
void ble_hs_test_util_create_conn(uint16_t handle, uint8_t *addr);
void ble_hs_test_util_rx_ack(uint16_t opcode, uint8_t status);
void ble_hs_test_util_rx_le_ack(uint16_t ocf, uint8_t status);
int ble_hs_test_util_l2cap_rx_payload_flat(struct ble_hs_conn *conn,
                                           struct ble_l2cap_chan *chan,
                                           const void *data, int len);

void ble_hs_test_util_init(void);

#endif
