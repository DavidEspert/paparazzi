(*
 * XML preprocessing for UBX protocol
 *
 * Copyright (C) 2003 Pascal Brisset, Antoine Drouin
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 *)

open Printf

let out = stdout

let sizeof = function
"U4" | "I4" -> 4
  | "U2" | "I2" -> 2
  | "U1" | "I1" -> 1
  | x -> failwith (sprintf "sizeof: unknown format '%s'" x)

let (+=) = fun r x -> r := !r + x

let c_type = fun format ->
  match format with
      "I2" -> "int16_t"
    | "I4" -> "int32_t"
    | "U2" -> "uint16_t"
    | "U4" -> "uint32_t"
    | "U1" -> "uint8_t"
    | "I1" -> "int8_t"
    | _ -> failwith (sprintf "Gen_ubx.c_type: unknown format '%s'" format)

let get_at = fun offset format block_size ->
  let t = c_type format in
  let block_offset =
    if block_size = 0 then "" else sprintf "+%d*_ubx_block" block_size in
  match format with
      "U4" | "I4" -> sprintf "(%s)(*((uint8_t*)_ubx_payload+%d%s)|*((uint8_t*)_ubx_payload+1+%d%s)<<8|((%s)*((uint8_t*)_ubx_payload+2+%d%s))<<16|((%s)*((uint8_t*)_ubx_payload+3+%d%s))<<24)" t offset block_offset offset block_offset t offset block_offset t offset block_offset
    | "U2" | "I2" -> sprintf "(%s)(*((uint8_t*)_ubx_payload+%d%s)|*((uint8_t*)_ubx_payload+1+%d%s)<<8)" t offset block_offset offset block_offset
    | "U1" | "I1" -> sprintf "(%s)(*((uint8_t*)_ubx_payload+%d%s))" t offset block_offset
    | _ -> failwith (sprintf "Gen_ubx.c_type: unknown format '%s'" format)

let define = fun x y ->
  fprintf out "#define %s %s\n" x y

exception Length_error of Xml.xml*int*int






  (** Generating send function ------------------------------------------------------------------- *)
let send_function = fun class_name m ->
  let msg_name = Xml.attrib m "name" in

  fprintf out "\n";
  fprintf out "//  --- UBX_%s_%s ---\n" class_name msg_name;

  (*  --> message identifier *)
  let msg_id = sprintf "UBX_%s_%s_ID" class_name msg_name in
  fprintf out "#define %s        %s\n" msg_id (Xml.attrib m "ID");

  (*  --> message priority *)
  let msg_pty = sprintf "UBX_%s_%s_PRIORITY" class_name msg_name in
  fprintf out "#define %s  0\n\n" msg_pty;

  let msg_len = sprintf "UBX_DATA_%s_%s_LENGTH" class_name msg_name in

  let field_name = fun f -> ExtXml.attrib f "name" in
  let format = fun f -> Xml.attrib f "format" in

  let param_name = fun f -> String.lowercase (field_name f) in
  let rec param_names = fun f r ->
    if Xml.tag f = "field" then
      param_name f :: r
    else
      List.fold_right param_names (Xml.children f) r in
  let param_type = fun f -> c_type (format f) in
  (*  --> calculate message size. If 0, simplify send_function *)
  let msg_size = ref 0 in
  let rec message_size = fun f ->
    match Xml.tag f with
        "field" ->
          let format = fun f -> Xml.attrib f "format" in
          let fmt = format f  in
          msg_size+= sizeof fmt
      | "block" ->
          List.iter message_size (Xml.children f)
      | _ -> assert (false)
  in
  List.iter message_size (Xml.children m);

  (*  --> function header *)
 if !msg_size <> 0 then begin
  (*fprintf out "static inline void ubx_send_%s_%s(struct device *dev, " class_name msg_name;*)
  fprintf out "static inline void ubx_send_%s_%s(" class_name msg_name;
  let comma = ref "" in
  let rec function_field_header = fun f ->
    match Xml.tag f with
        "field" ->
          let p = param_name f in
          let t = param_type f in
          fprintf out "%sconst %s _%s" !comma t p;
          comma := ", "
      | "block" ->
        List.iter function_field_header (Xml.children f)
      | _ -> assert (false) in
  let print_parameters_function_header = function
  [] -> ()
    | f::fields ->
      function_field_header f;
      List.iter (function_field_header) fields in
  print_parameters_function_header (Xml.children m);
  fprintf out ") { \n";
 end else
  fprintf out "static inline void ubx_send_%s_%s(void) {\n" class_name msg_name;

  (* --> function initialization *)
  fprintf out "  uint8_t slot_idx;\n" ;
  fprintf out "\n" ;

  fprintf out "  _UBX_SEND_TRACE_(\"\\nubx_send_%s_%s:\\n\");\n" class_name msg_name;
  fprintf out "\n" ;
  
  (*  --> Get device buffer *)
  fprintf out "  /* 1.- try to get a slot in device's buffer */\n" ;
  fprintf out "  if(UBX_DEV_CHECK_FREE_SPACE( %s, &slot_idx)){\n" msg_len;
  fprintf out "    uint8_t *buff = UBX_DEV_GET_BUFFER_POINTER(slot_idx);\n";

  (*  --> Call ubx_header *)
  fprintf out "    /* 2.- set transport and message HEADERS (all in one) */\n" ;
  fprintf out "    ubx_header(buff, UBX_%s_ID, %s, %s);\n" class_name msg_id msg_len;

  (*  --> Call Ubx_data *)
  fprintf out "    /* 3.- set message DATA in buffer */\n" ;
 if !msg_size <> 0 then begin
  fprintf out "    ubx_data_%s_%s_pack((buff + sizeof(struct Ubx_Header))" class_name msg_name;
  let rec function_field_call = fun f ->
    match Xml.tag f with
        "field" ->
          let p = param_name f in
          fprintf out ", _%s" p
      | "block" ->
        List.iter function_field_call (Xml.children f)
      | _ -> assert (false) in
  List.iter function_field_call (Xml.children m);
  fprintf out ");\n";
 end else
  fprintf out "    /*ubx_data_%s_%s_pack((buff + sizeof(struct Ubx_Header)));*/\n" class_name msg_name;

  (*  --> Call Ubx_trailer *)
  fprintf out "    /* 4.- set transport TAIL in buffer */\n" ;
  fprintf out "    ubx_trailer(buff, %s);\n" msg_len;

  fprintf out "    /* 5.- send message */\n" ;
  fprintf out "    UBX_DEV_SEND_MESSAGE(slot_idx, %s);\n" msg_pty;
  fprintf out "  }\n";
  fprintf out "}\n"


let parse_class = fun c ->
  let _class_id = int_of_string (Xml.attrib c "id")
  and class_name = Xml.attrib c "name" in
  fprintf out "\n";
  fprintf out "// %s ----------------------------------------------------------------------\n" class_name;
  define (sprintf "UBX_%s_ID" class_name) (Xml.attrib c "ID");
  List.iter (send_function class_name) (Xml.children c)


let _ =
  if Array.length Sys.argv <> 2 then begin
    failwith (sprintf "Usage: %s <.xml ubx protocol file>" Sys.argv.(0))
  end;
  let xml_file = Sys.argv.(1) in
  try
    let xml = Xml.parse_file xml_file in
    fprintf out "/* Generated from %s */\n" xml_file;
    fprintf out "/* Please DO NOT EDIT */\n\n";

    fprintf out "#ifndef _UBX_PROTOCOL_H_\n";
    fprintf out "#define _UBX_PROTOCOL_H_\n\n";

    fprintf out "#include <string.h>        //required for memcpy\n";
    fprintf out "#include \"ubx_protocol_data.h\"\n\n";

    Printf.fprintf out "//#define _UBX_SEND_DEBUG_\n\n";
    Printf.fprintf out "#ifdef _UBX_SEND_DEBUG_\n#include <stdio.h> \n#define _UBX_SEND_TRACE_(...) fprintf (stderr, __VA_ARGS__); fflush(stdout);\n#else\n#define _UBX_SEND_TRACE_(...)\n#endif\n\n";

(*    define "UBX_SYNC1" "0xB5";
    define "UBX_SYNC2" "0x62";

    Printf.fprintf out "typedef struct {\n  const uint8_t sync1;\n  const uint8_t sinc2;\n  uint8_t nav_id;\n  uint8_t msg_id;\n  uint16_t length;\n}UbxMsgHeader;\n\nstatic UbxMsgHeader msg_hd = { .sync1 = UBX_SYNC1, .sync2 = UBX_SYNC2 };\n\n\n";*)

    (** Generating auxiliar macros ------------------------------------------------------------------- *)
    fprintf out "\n\n// Auxiliar macros and functions ----------------------------------------------------------------------\n";
(*    let device = sprintf "device.h"  in
    let chck_fs = sprintf "DEV_GPS##->checkFreeSpace(_x+8, _y)"  in
    let get_bp = sprintf "DEV_GPS##->get_buff_pointer(_x)"  in
    let send_msg = sprintf "DEV_GPS##->sendMessage(_x, _y)"  in*)

    let device = sprintf "uart.h"  in
    let chck_fs = sprintf "GpsLink(CheckFreeSpace(_x+8, _y))"  in
    let get_bp = sprintf "GpsLink(Get_buff_pointer(_x))"  in
    let send_msg = sprintf "GpsLink(SendMessage(_x, _y))"  in
    let auxiliar_macros_device = fun device chck_fs get_bp send_msg ->
      fprintf out "#include \"mcu_periph/%s\"\n" device;
      fprintf out "\n";
      if (device <> "device.h") then
        fprintf out "#define __GpsLink(dev, _x) dev##_x\n#define _GpsLink(dev, _x)  __GpsLink(dev, _x)\n#define GpsLink(_x) _GpsLink(GPS_LINK, _x)\n"
      else
        fprintf out "#define dev_gps(_x) dev_##_x\n#define DEV_GPS     dev_gps(GPS_LINK)\n";
      fprintf out "\n";
      fprintf out "//Check free space on device's buffer: transport header + message header + data + tail = 2 + 4 + _x + 2 = _x + 8\n";
      fprintf out "#define UBX_DEV_CHECK_FREE_SPACE( _x, _y)  %s\n" chck_fs;
      fprintf out "//Get pointer to the assigned slot in device's buffer.\n";
      fprintf out "#define UBX_DEV_GET_BUFFER_POINTER(_x)     %s\n" get_bp;
      fprintf out "#define UBX_DEV_SEND_MESSAGE(_x, _y)       %s\n" send_msg in
    auxiliar_macros_device device chck_fs get_bp send_msg;

    fprintf out "\n";

    fprintf out "#define UBX_SYNC1 0xB5\n#define UBX_SYNC2 0x62\n";
    fprintf out "\n";
    
    fprintf out "struct Ubx_Header {\n  //Transport header\n  const uint8_t sync1;\n  const uint8_t sync2;\n  //Message header\n  uint8_t nav_id;\n  uint8_t msg_id;\n  uint16_t length;\n};\n\nextern struct Ubx_Header ubx_hd;\n";
    fprintf out " \n";

    fprintf out "static inline void ubx_header(uint8_t *buff, uint8_t _nav_id, uint8_t _msg_id, uint16_t _length) {\n";
    fprintf out "  ubx_hd.nav_id = _nav_id;\n  ubx_hd.msg_id = _msg_id;\n  ubx_hd.length = _length;\n\n  memcpy(buff, &ubx_hd, sizeof(struct Ubx_Header));\n}\n";
    fprintf out "\n";

    fprintf out "static inline void ubx_trailer(uint8_t *buff, uint16_t _length) {
  uint16_t ck_a, ck_b;
  uint16_t i;
  //calculate checksum over 'Message header' and 'data'
  ck_a = ck_b = 0;
  for(i = 2; i < 6 +_length; i++) {
    ck_a += buff[i];
    ck_b += ck_a;
  }
  buff[i++] = ck_a;
  buff[i] = ck_b;
}\n";
    fprintf out "\n";

    (** Generating ubx_send_FUNCTIONS ------------------------------------------------------------------- *)
    List.iter parse_class (Xml.children xml);

    fprintf out "#endif// _UBX_PROTOCOL_H_\n"
  with
      Xml.Error (em, ep) ->
        let l = Xml.line ep
        and c1, c2 = Xml.range ep in
        fprintf stderr "File \"%s\", line %d, characters %d-%d:\n" xml_file l c1 c2;
        fprintf stderr "%s\n" (Xml.error_msg em);
        exit 1
    | Length_error (m, l1, l2) ->
      fprintf stderr "File \"%s\", inconsistent length: %d expected, %d found from fields in message:\n %s\n" xml_file l1 l2 (Xml.to_string_fmt m);
      exit 1
    | Dtd.Check_error e ->
      fprintf stderr "File \"%s\", DTD check error: %s\n" xml_file (Dtd.check_error e)
    | Dtd.Prove_error e ->
      fprintf stderr "\nFile \"%s\", DTD check error: %s\n\n" xml_file (Dtd.prove_error e)
