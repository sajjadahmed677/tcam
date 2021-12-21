// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// tl_main package generated by `tlgen.py` tool

package tl_main_pkg;

  localparam logic [31:0] ADDR_SPACE_ICCM       = 32'h 40000000;
  localparam logic [31:0] ADDR_SPACE_DCCM       = 32'h 80000000;
  localparam logic [31:0] ADDR_SPACE_TCAM       = 32'h 00000000;
  localparam logic [31:0] ADDR_SPACE_UART0      = 32'h c0000000;


  localparam logic [31:0] ADDR_MASK_ICCM        = 32'h c0000000;
  localparam logic [31:0] ADDR_MASK_DCCM        = 32'h c0000000;
  localparam logic [31:0] ADDR_MASK_TCAM        = 32'h c0000000;
  localparam logic [31:0] ADDR_MASK_UART0       = 32'h c0000000;

  localparam int N_HOST   = 2;
  localparam int N_DEVICE = 4;

  typedef enum int {
    TlIccm = 0,
    TlDccm = 1,
    TlTcam = 2,
    TlUart0 = 3
  } tl_device_e;

  typedef enum int {
    TlBrqif = 0,
    TlBrqlsu = 1
  } tl_host_e;

endpackage