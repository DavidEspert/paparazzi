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
let send_function = fun f_type class_name m ->
  let eol = ref "" in
  let declaration = ref "" in
  let void = ref "" in
  if (f_type <> "FUNCTION") then begin
    eol := " \\\n";
    declaration := "#define";
  end else begin
    eol := "\n";
    declaration := "static inline void";
    void := "void";
  end;
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
  fprintf out "%s ubx_send_%s_%s(" !declaration class_name msg_name;
  let comma = ref "" in
  let rec function_field_header = fun f ->
    match Xml.tag f with
        "field" ->
          let p = param_name f in
          let t = param_type f in
          if (f_type <> "FUNCTION") then
            fprintf out "%s_%s" !comma p
          else
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
  fprintf out ") {%s" !eol;
 end else
  fprintf out "%s ubx_send_%s_%s(%s) {%s" !declaration class_name msg_name !void !eol;

  (* --> function initialization *)
  fprintf out "  uint8_t trans_len = UBX_DEV_TRANSACTION_LEN();%s" !eol;
  fprintf out "  uint8_t hd_len = 6; /* sizeof(struct Ubx_Header); */%s" !eol;
  fprintf out "  uint8_t tl_len = 2;%s" !eol;
  fprintf out "  uint8_t dev_slot;%s" !eol;
  fprintf out "  uint8_t buff_slot;%s" !eol;
  fprintf out "%s" !eol;

  fprintf out "  _UBX_SEND_TRACE_(\"\\nubx_send_%s_%s:\\n\");%s" class_name msg_name !eol;
  fprintf out "%s" !eol;
  
  (*  --> Get device slot *)
  fprintf out "  /* 1.- try to get a device's 'transaction' slot */%s" !eol;
  fprintf out "  if(UBX_DEV_CHECK_FREE_SPACE(&dev_slot)){%s" !eol;

  (*  --> Get buffer slot *)
  fprintf out "    /* 2.- try to get a slot in dynamic buffer */%s" !eol;
  fprintf out "    if( dynamic_buffer_check_free_space(&dynamic_buff, (trans_len + hd_len + %s + tl_len), &buff_slot) ){%s" msg_len !eol;
  fprintf out "      /* 3.- get buffer pointer */%s" !eol;
  fprintf out "      uint8_t *buff = dynamic_buffer_get_slot_pointer(&dynamic_buff, buff_slot);%s" !eol;

  (*  --> Set transaction *)
  fprintf out "      /* 4.- set transaction */%s" !eol;
  fprintf out "      UBX_DEV_TRANSACTION_PACK(buff, (buff + trans_len), %s, &ubx_callback);%s" msg_len !eol;

  (*  --> Call ubx_header *)
  fprintf out "      /* 5.- set transport and message HEADERS (all in one) */%s" !eol;
  fprintf out "      ubx_header(buff + trans_len, UBX_%s_ID, %s, %s);%s" class_name msg_id msg_len !eol;

  (*  --> Call Ubx_data *)
  fprintf out "      /* 6.- set message DATA in buffer */%s" !eol;
 if !msg_size <> 0 then begin
  fprintf out "      ubx_data_%s_%s_pack((buff + trans_len + hd_len)" class_name msg_name;
  let rec function_field_call = fun f ->
    match Xml.tag f with
        "field" ->
          let p = param_name f in
          fprintf out ", _%s" p
      | "block" ->
        List.iter function_field_call (Xml.children f)
      | _ -> assert (false) in
  List.iter function_field_call (Xml.children m);
  fprintf out ");%s" !eol;
 end else
  fprintf out "      /*ubx_data_%s_%s_pack((buff + trans_len + hd_len));*/%s" class_name msg_name !eol;

  (*  --> Call Ubx_trailer *)
  fprintf out "      /* 7.- set transport TAIL in buffer */%s" !eol;
  fprintf out "      ubx_trailer(buff + trans_len, %s);%s" msg_len !eol;

  fprintf out "      /* 8.- send message */%s" !eol;
  fprintf out "      UBX_DEV_SEND_MESSAGE(dev_slot, buff, %s);%s" msg_pty !eol;
  fprintf out "    }%s" !eol;
  fprintf out "    else {%s" !eol;
  fprintf out "      /* 9.- release device's slot */%s" !eol;
  fprintf out "      UBX_DEV_FREE_SPACE(dev_slot);%s" !eol;
  fprintf out "    }%s" !eol;
  fprintf out "  }%s" !eol;
  fprintf out "}\n"

let parse_class = fun c ->
  let _class_id = int_of_string (Xml.attrib c "id")
  and class_name = Xml.attrib c "name" in
  fprintf out "\n";
  fprintf out "// %s ----------------------------------------------------------------------\n" class_name;
  define (sprintf "UBX_%s_ID" class_name) (Xml.attrib c "ID");
(*  List.iter (send_function "MACRO" class_name) (Xml.children c)*)
  List.iter (send_function "FUNCTION" class_name) (Xml.children c)

let auxiliar_macros_device = fun h ->
(*    let chck_fs = sprintf "GpsLink(check_free_space(_x))"  in
    let free_space = sprintf "GpsLink(free_space(_x))" in
    let trans_len = sprintf "GpsLink(transaction_len())" in
    let trans_pack = sprintf "GpsLink(transaction_pack(_x, _y, _w, _z))" in
    let send_msg = sprintf "GpsLink(sendMessage(_x, _y, _z))"  in*)

    let chck_fs = sprintf "GpsLink(CheckFreeSpace(_x))"  in
    let free_space = sprintf "GpsLink(FreeSpace(_x))" in
    let trans_len = sprintf "GpsLink(TransactionLen())" in
    let trans_pack = sprintf "GpsLink(TransactionPack(_x, _y, _w, _z))" in
    let send_msg = sprintf "GpsLink(SendMessage(_x, _y, _z))"  in
    let print_auxiliar_macros_device = fun chck_fs send_msg free_space trans_pack ->
      fprintf h "#include \"mcu_periph/uart.h\"\n";
      fprintf h "\n";
      fprintf h "#define __GpsLink(dev, _x)  dev##_x\n#define _GpsLink(dev, _x)   __GpsLink(dev, _x)\n#define GpsLink(_x)         _GpsLink(GPS_LINK, _x)\n";
(*      fprintf h "#define __GpsLink(dev, _x)  dl_##dev##->##_x\n#define _GpsLink(dev, _x)   __GpsLink(dev, _x)\n#define GpsLink(_x)         _GpsLink(GPS_LINK, _x)\n";*)
      fprintf h "\n";
      fprintf h "#define UBX_DEV_CHECK_FREE_SPACE( _x)                %s\n" chck_fs;
      fprintf h "#define UBX_DEV_FREE_SPACE(_x)                       %s\n" free_space;
      fprintf h "#define UBX_DEV_TRANSACTION_LEN()                    %s\n" trans_len;
      fprintf h "#define UBX_DEV_TRANSACTION_PACK(_x, _y, _w, _z)     %s\n" trans_pack;
      fprintf h "#define UBX_DEV_SEND_MESSAGE(_x, _y, _z)             %s\n" send_msg in
    print_auxiliar_macros_device chck_fs send_msg free_space trans_pack

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
    fprintf out "#include \"ubx_protocol_data.h\"\n";
    fprintf out "#include \"mcu_periph/dynamic_buffer.h\"\n\n";

    Printf.fprintf out "//#define _UBX_SEND_DEBUG_\n\n";
    Printf.fprintf out "#ifdef _UBX_SEND_DEBUG_\n#include <stdio.h> \n#define _UBX_SEND_TRACE_(...) fprintf (stderr, __VA_ARGS__); fflush(stdout);\n#else\n#define _UBX_SEND_TRACE_(...)\n#endif\n\n";

    (** Generating auxiliar macros ------------------------------------------------------------------- *)
    fprintf out "\n\n// Auxiliar macros and functions ----------------------------------------------------------------------\n";
    auxiliar_macros_device out;

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

    fprintf out "extern void ubx_callback(void *slot_ptr);\n";
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
