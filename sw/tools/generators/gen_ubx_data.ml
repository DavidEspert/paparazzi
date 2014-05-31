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




let parse_message = fun class_name m ->
  let msg_name = Xml.attrib m "name" in
  let field_name = fun f -> ExtXml.attrib f "name" in
  let format = fun f -> Xml.attrib f "format" in

  fprintf out "//  --- UBX_%s_%s ---\n" class_name msg_name;

  (*print 'field access' macros *)
  let offset = ref 0 in
  let rec gen_access_macro = fun block_size f ->
    match Xml.tag f with
        "field" ->
          let fn = field_name f
          and fmt = format f  in
          let block_no = if block_size = 0 then "" else ",_ubx_block" in
          define (sprintf "UBX_%s_%s_%s(_ubx_payload%s)" class_name msg_name fn block_no) (get_at !offset fmt block_size);
          offset += sizeof fmt
      | "block" ->
        let s = int_of_string (Xml.attrib f "length") in
        let o = !offset in
        List.iter (gen_access_macro s) (Xml.children f);
        let s' = !offset - o in
        if s <> s' then raise (Length_error (f, s, s'))
      | x -> failwith ("Unexpected field: " ^ x)
  in
  List.iter (gen_access_macro 0) (Xml.children m);
  begin
    try
      let l = int_of_string (Xml.attrib m "length") in
      if l <> !offset then raise (Length_error (m, l, !offset))
    with
        Xml.No_attribute("length") -> () (** Undefined length authorized *)
  end;

  (*prepare for printing 'ubx_data' struct and function *)
  let param_name = fun f -> String.lowercase (field_name f) in
  let rec param_names = fun f r ->
    if Xml.tag f = "field" then
      param_name f :: r
    else
      List.fold_right param_names (Xml.children f) r in
  let param_type = fun f -> c_type (format f) in

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

 if !msg_size <> 0 then begin
  fprintf out "\n";
  (*print 'struct' *)
  fprintf out "struct ubx_data_%s_%s {\n"class_name msg_name;
  let rec struct_field = fun f ->
    match Xml.tag f with
        "field" ->
          let p = param_name f in
          let t = param_type f in
          fprintf out "  %s %s;\n" t p
      | "block" ->
        List.iter struct_field (Xml.children f)
      | _ -> assert (false) in
  List.iter struct_field (Xml.children m);
  fprintf out "};\n\n";

  (*print 'ubx_data' function *)
  fprintf out "#define UBX_DATA_%s_%s_LENGTH   (%u)\n" class_name msg_name !msg_size;

  fprintf out "static inline void ubx_data_%s_%s_pack(uint8_t *buff" class_name msg_name;
  let rec function_field_header = fun f ->
    match Xml.tag f with
        "field" ->
          let p = param_name f in
          let t = param_type f in
          fprintf out ", const %s _%s" t p
      | "block" ->
        List.iter function_field_header (Xml.children f)
      | _ -> assert (false) in
  List.iter function_field_header (Xml.children m);
  fprintf out ") { \n";

  fprintf out "  struct ubx_data_%s_%s     packet;\n\n" class_name msg_name;

  let rec function_field_body = fun f ->
    match Xml.tag f with
        "field" ->
          let p = param_name f in
          fprintf out "  packet.%s = _%s;\n" p p
      | "block" ->
        List.iter function_field_body (Xml.children f)
      | _ -> assert (false) in
  List.iter function_field_body (Xml.children m);

  fprintf out "\n  memcpy(buff, &packet, UBX_DATA_%s_%s_LENGTH);\n" class_name msg_name;
  fprintf out "}\n\n"
 end else begin
  fprintf out "#define UBX_DATA_%s_%s_LENGTH   (%u)\n" class_name msg_name !msg_size;
  fprintf out "/*static inline void ubx_data_%s_%s_pack(uint8_t *buff){}*/\n\n" class_name msg_name
 end


let parse_class = fun c ->
  let _class_id = int_of_string (Xml.attrib c "id")
  and class_name = Xml.attrib c "name" in

  fprintf out "\n";
  fprintf out "//- %s ----------------------------------------------------------------------\n" class_name;
  List.iter (parse_message class_name) (Xml.children c)


let _ =
  if Array.length Sys.argv <> 2 then begin
    failwith (sprintf "Usage: %s <.xml ubx protocol file>" Sys.argv.(0))
  end;
  let xml_file = Sys.argv.(1) in
  try
    let xml = Xml.parse_file xml_file in
    fprintf out "/* Generated from %s */\n" xml_file;
    fprintf out "/* Please DO NOT EDIT */\n\n";

    List.iter parse_class (Xml.children xml)
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
