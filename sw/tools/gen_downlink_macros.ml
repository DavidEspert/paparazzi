(*
 * XML preprocessing of messages.xml for downlink protocol
 *
 * Copyright (C) 2003-2008 ENAC, Pascal Brisset, Antoine Drouin
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

type format = string

type _type =
    Basic of string
  | Array of string * string

let c_type = fun format ->
  match format with
      "Float" -> "float"
    | "Double" -> "double"
    | "Int32" -> "int32_t"
    | "Int16" -> "int16_t"
    | "Int8" -> "int8_t"
    | "Uint32" -> "uint32_t"
    | "Uint16" -> "uint16_t"
    | "Uint8" -> "uint8_t"
    | _ -> failwith (sprintf "gen_messages.c_type: unknown format '%s'" format)

let dl_type = fun format ->
  match format with
      "Float" -> "DL_TYPE_FLOAT"
    | "Double" -> "DL_TYPE_DOUBLE"
    | "Int32" -> "DL_TYPE_INT32"
    | "Int16" -> "DL_TYPE_INT16"
    | "Int8" -> "DL_TYPE_INT8"
    | "Uint32" -> "DL_TYPE_UINT32"
    | "Uint16" -> "DL_TYPE_UINT16"
    | "Uint8" -> "DL_TYPE_UINT8"
    | _ -> failwith (sprintf "gen_messages.c_type: unknown format '%s'" format)

type field = _type  * string * format option

type fields = field list

type message = {
  name : string;
  id : int;
  period : float option;
  fields : fields
}

module Syntax = struct
  (** Parse a type name and returns a _type value *)
  let parse_type = fun t varname ->
    let n = String.length t in
    if n >=2 && String.sub t (n-2) 2 = "[]" then
      Array (String.sub t 0 (n-2), varname)
    else
      Basic t

  let length_name = fun s -> s^"_len"

  let assoc_types t =
    try
      List.assoc t Pprz.types
    with
        Not_found ->
          failwith (sprintf "Error: '%s' unknown type" t)

  let rec sizeof = function
  Basic t -> string_of_int (assoc_types t).Pprz.size
    | Array (t, varname) -> sprintf "%s*%s" (length_name varname) (sizeof (Basic t))

  let rec nameof = function
  Basic t -> String.capitalize t
    | Array _ -> failwith "nameof"

  (** Translates a "message" XML element into a value of the 'message' type *)
  let struct_of_xml = fun xml ->
    let name = ExtXml.attrib xml "name"
    and id = ExtXml.int_attrib xml "id"
    and period = try Some (ExtXml.float_attrib xml "period") with _ -> None
    and fields =
      List.map
        (fun field ->
          let id = ExtXml.attrib field "name"
          and type_name = ExtXml.attrib field "type"
          and fmt = try Some (Xml.attrib field "format") with _ -> None in
          let _type = parse_type type_name id in
          (_type, id, fmt))
        (Xml.children xml) in
    { id=id; name = name; period = period; fields = fields }

  let check_single_ids = fun msgs ->
    let tab = Array.create 256 false
    and  last_id = ref 0 in
    List.iter (fun msg ->
      if tab.(msg.id) then
        failwith (sprintf "Duplicated message id: %d" msg.id);
      if msg.id < !last_id then
        fprintf stderr "Warning: unsorted id: %d\n%!" msg.id;
      last_id := msg.id;
      tab.(msg.id) <- true)
      msgs

  (** Translates one class of a XML message file into a list of messages *)
  let read = fun filename class_ ->
    let xml = Xml.parse_file filename in
    try
      let xml_class = ExtXml.child ~select:(fun x -> Xml.attrib x "name" = class_) xml "class" in
      let msgs = List.map struct_of_xml (Xml.children xml_class) in
      check_single_ids msgs;
      msgs
    with
        Not_found -> failwith (sprintf "No class '%s' found" class_)
end (* module Suntax *)


module Gen_onboard = struct
(** SIZE functions *)
  let rec size_fields = fun fields size ->
    match fields with
      (Basic t, _, _)::fields -> size_fields fields (size + int_of_string(Syntax.sizeof (Basic t)))
      | (Array (t,varname), _, _)::fields -> (string_of_int(size+1)^"+_"^Syntax.sizeof (Array(t, varname)))
      | [] -> string_of_int(size)

  let size_of_message = fun m -> size_fields m.fields 0

  let estimated_size_of_message = fun m ->
    try
      List.fold_right
        (fun (t, _, _)  r ->  int_of_string(Syntax.sizeof t)+r)
        m.fields 0
    with
        Failure "int_of_string" -> (-1)

(** PRINT functions *)
  (* Prints parameters in function header *)
  let print_parameter_function_header h = function
  (Array (t, varname), s, _) -> fprintf h "const uint8_t _%s, const %s *_%s" (Syntax.length_name s) (c_type (Syntax.nameof (Basic t))) s
    | (t, s, _) -> fprintf h "const %s *_%s" (c_type (Syntax.nameof t)) s

  let print_parameters_function_header h = function
  [] -> ()
    | f::fields ->
      print_parameter_function_header h f;
      List.iter (fun f -> fprintf h ", "; print_parameter_function_header h f) fields

  (* Prints parameters in macro header *)
  let print_parameter_macro_header h = function
  (Array (t, varname), s, _) -> fprintf h "_%s, _%s" (Syntax.length_name s) s
    | (t, s, _) -> fprintf h "_%s" s

  let print_parameters_macro_header h = function
  [] -> ()
    | f::fields ->
      print_parameter_macro_header h f;
      List.iter (fun f -> fprintf h ", "; print_parameter_macro_header h f) fields


  (** Prints downlink macro *)
  let print_downlink_macro = fun h f_type trans_type trans_name dev_type dev_name eol message(*{name=s; fields = fields}*) ->
    let s = message.name in
    let fields = message.fields in
    let data_len1 = ("DOWNLINK_DATA_"^s^"_LENGTH" ) in
    let data_len2 = "("^(size_of_message message)^")" in

    fprintf h "// %s ----------------------------------\n" s;
    if List.length fields > 0 then begin
      fprintf h "%s downlink_send_%s(%s%s, %s%s, " f_type s trans_type trans_name dev_type dev_name;
      if (f_type <> "#define") then
        print_parameters_function_header h fields
      else
        print_parameters_macro_header h fields;
      fprintf h "){%s" eol
    end else
      fprintf h "%s downlink_send_%s(%s%s, %s%s){%s" f_type s trans_type trans_name dev_type dev_name eol;

    fprintf h "\tuint8_t buff_idx;%s" eol;
    (*fprintf h "\tuint8_t temp_buffer[256];%s" eol;*)
    if (f_type <> "#define") then begin
      fprintf h "\tuint8_t hd_len = %s->header_len();%s" trans_name eol;
      fprintf h "\tuint8_t tl_len = %s->tail_len();%s" trans_name eol;
    end else begin
      fprintf h "\tuint8_t hd_len = %s##_header_len();%s" trans_name eol;
      fprintf h "\tuint8_t tl_len = %s##_tail_len();%s" trans_name eol;
    end;
    fprintf h "%s" eol;

(** DEBUG init *)
    fprintf h "\tprintf(\"\\nDOWNLINK_SEND_%s:\\n\");%s" s eol;
    (*fprintf h "\tprintf(\"\\nDOWNLINK_SEND_%s (id %%d):\\n\", DL_%s);%s" s s eol;*)
    fprintf h "%s" eol;
(** DEBUG end*)

(*    fprintf h "\t/* 1.- Check if there is space enough in device's buffer */%s" eol;
    if (f_type <> "#define") then
      fprintf h "\tif(dev->checkFreeSpace(%s + hd_len + tl_len)){%s" data_len1 eol
    else
      fprintf h "\tif(dev->checkFreeSpace(%s + hd_len + tl_len)){%s" data_len2 eol;
    fprintf h "%s" eol;

    fprintf h "\t  /* 2.- set message HEADER in temporary buffer (in depends on transport layer) */%s" eol;
    if (f_type <> "#define") then
      fprintf h "\t  %s->header(temp_buffer, %s, DL_%s);%s" trans_name data_len1 s eol
    else
      fprintf h "\t  %s##_header(temp_buffer, %s, DL_%s);%s" trans_name data_len2 s eol;
    fprintf h "%s" eol;

    fprintf h "\t  /* 3.- set message DATA in temporary buffer */%s" eol;
    if List.length fields > 0 then begin
      fprintf h "\t  downlink_data_%s_pack((temp_buffer+hd_len), " s;
      print_parameters_macro_header h fields;
      fprintf h ");%s" eol;
    end;
    fprintf h "%s" eol;

    fprintf h "\t  /* 4.- set message TAIL in temporary buffer (in depends on transport layer) */%s" eol;
    if (f_type <> "#define") then
      fprintf h "\t  %s->tail(temp_buffer, %s);%s" trans_name data_len1 eol
    else
      fprintf h "\t  %s##_tail(tem_buffer, %s);%s" trans_name data_len2 eol;
    fprintf h "%s" eol;

    fprintf h "\t  /* 5.- put message in device's buffer */%s" eol;
    if (f_type <> "#define") then
      fprintf h "\t  for(uint8_t i = 0; i < (%s + hd_len + tl_len); i++)%s" data_len1 eol
    else
      fprintf h "\t  for(uint8_t i = 0; i < (%s + hd_len + tl_len); i++)%s" data_len2 eol;
    fprintf h "\t    dev->transmit(temp_buffer[i]);%s" eol;
    fprintf h "\t}%s" eol;
    fprintf h "}\n\n"*)

    fprintf h "\t/* 1.- Try to get a slot in device's buffer */%s" eol;
    if (f_type <> "#define") then
      fprintf h "\tif(dev->get_tx_slot((%s + hd_len + tl_len), &buff_idx) == TRUE){%s" data_len1 eol
    else
      fprintf h "\tif(dev->get_tx_slot((%s + hd_len + tl_len), &buff_idx) == TRUE){%s" data_len2 eol;
    fprintf h "\t  uint8_t *buff = dev->get_buff_pointer(buff_idx);%s" eol;
    fprintf h "%s" eol;

    fprintf h "\t  /* 2.- set message HEADER in buffer (in depends on transport layer) */%s" eol;
    if (f_type <> "#define") then
      fprintf h "\t  %s->header(buff, %s, DL_%s);%s" trans_name data_len1 s eol
    else
      fprintf h "\t  %s##_header(buff, %s, DL_%s);%s" trans_name data_len2 s eol;
    fprintf h "%s" eol;

    fprintf h "\t  /* 3.- set message DATA in buffer */%s" eol;
    if List.length fields > 0 then begin
      fprintf h "\t  downlink_data_%s_pack((buff+hd_len), " s;
      print_parameters_macro_header h fields;
      fprintf h ");%s" eol;
    end;
    fprintf h "%s" eol;

    fprintf h "\t  /* 4.- set message TAIL in buffer (in depends on transport layer) */%s" eol;
    if (f_type <> "#define") then
      fprintf h "\t  %s->tail(buff, %s);%s" trans_name data_len1 eol
    else
      fprintf h "\t  %s##_tail(buff, %s);%s" trans_name data_len2 eol;
    fprintf h "\t  dev->sendMessage(buff_idx, 0);%s" eol;
    fprintf h "\t}%s" eol;
    fprintf h "}\n\n"


  let print_null_downlink_macro = fun h f_type trans_type trans_name dev_type dev_name eol {name=s; fields = fields} ->
    if List.length fields > 0 then begin
      fprintf h "%s DOWNLINK_SEND_%s(%s%s, %s%s, " f_type s trans_type trans_name dev_type dev_name;
      if (f_type <> "#define") then
        print_parameters_function_header h fields
      else
        print_parameters_macro_header h fields;
      fprintf h "){}%s" eol
    end else
      fprintf h "%s DOWNLINK_SEND_%s(%s%s, %s%s){}%s" f_type s trans_type trans_name dev_type dev_name eol

    (*if List.length fields > 0 then begin
      fprintf h "void DOWNLINK_SEND_%s(struct DownlinkTransport *tp, " s;
    end else
      fprintf h "void DOWNLINK_SEND_%s(struct DownlinkTransport *tp" s;
    print_parameters_function_header h fields;
    fprintf h ") {}\n"*)

  (** Prints the messages ids *)
  let print_enum = fun h class_ messages ->
    List.iter (fun m ->
      if m.id < 0 || m.id > 255 then begin
        fprintf stderr "Error: message %s has id %d but should be between 0 and 255\n" m.name m.id; exit 1;
      end
      else fprintf h "#define DL_%s %d\n" m.name m.id
    ) messages;
    fprintf h "#define DL_MSG_%s_NB %d\n\n" class_ (List.length messages)

  (** Prints the macros required to send a message 
  let print_null_downlink_macros = fun h messages ->
    List.iter (print_null_downlink_macro h) messages*)

  (** Prints the macro to get access to the fields of a received message *)
  let print_get_macros = fun h check_alignment message ->
    let msg_name = message.name in
    let offset = ref Pprz.offset_fields in

    (** Prints the macro for one field, using the global [offset] ref *)
    let parse_field = fun (_type, field_name, _format) ->
      if !offset < 0 then
        failwith "FIXME: No field allowed after an array field (print_get_macros)";
      (** Converts bytes into the required type *)
      let typed = fun o pprz_type -> (* o for offset *)
        let size = pprz_type.Pprz.size in
        if check_alignment && o mod (min size 4) <> 0 then
          failwith (sprintf "Wrong alignment of field '%s' in message '%s" field_name msg_name);

        match size with
            1 -> sprintf "(%s)(*((uint8_t*)_payload+%d))" pprz_type.Pprz.inttype o
          | 2 -> sprintf "(%s)(*((uint8_t*)_payload+%d)|*((uint8_t*)_payload+%d+1)<<8)" pprz_type.Pprz.inttype o o
          | 4 when pprz_type.Pprz.inttype = "float" ->
            sprintf "({ union { uint32_t u; float f; } _f; _f.u = (uint32_t)(*((uint8_t*)_payload+%d)|*((uint8_t*)_payload+%d+1)<<8|((uint32_t)*((uint8_t*)_payload+%d+2))<<16|((uint32_t)*((uint8_t*)_payload+%d+3))<<24); _f.f; })" o o o o
          | 8 when pprz_type.Pprz.inttype = "double" ->
            let s = ref (sprintf "*((uint8_t*)_payload+%d)" o) in
            for i = 1 to 7 do
              s := !s ^ sprintf "|((uint64_t)*((uint8_t*)_payload+%d+%d))<<%d" o i (8*i)
            done;

            sprintf "({ union { uint64_t u; double f; } _f; _f.u = (uint64_t)(%s); Swap32IfBigEndian(_f.u); _f.f; })" !s
          | 4 ->
            sprintf "(%s)(*((uint8_t*)_payload+%d)|*((uint8_t*)_payload+%d+1)<<8|((uint32_t)*((uint8_t*)_payload+%d+2))<<16|((uint32_t)*((uint8_t*)_payload+%d+3))<<24)" pprz_type.Pprz.inttype o o o o
          | _ -> failwith "unexpected size in Gen_messages.print_get_macros" in

      (** To be an array or not to be an array: *)
      match _type with
          Basic t ->
            let pprz_type = Syntax.assoc_types t in
            fprintf h "#define DL_%s_%s(_payload) (%s)\n" msg_name field_name (typed !offset pprz_type);
            offset := !offset + pprz_type.Pprz.size

        | Array (t, _varname) ->
      (** The macro to access to the length of the array *)
          fprintf h "#define DL_%s_%s_length(_payload) (%s)\n" msg_name field_name (typed !offset (Syntax.assoc_types "uint8"));
          incr offset;

      (** The macro to access to the array itself *)
          let pprz_type = Syntax.assoc_types t in
          if check_alignment && !offset mod (min pprz_type.Pprz.size 4) <> 0 then
            failwith (sprintf "Wrong alignment of field '%s' in message '%s" field_name msg_name);

          fprintf h "#define DL_%s_%s(_payload) ((%s*)(_payload+%d))\n" msg_name field_name pprz_type.Pprz.inttype !offset;
          offset := -1 (** Mark for no more fields *)
    in

    fprintf h "\n";
    (** Do it for all the fields of the message *)
    List.iter parse_field message.fields

end (* module Gen_onboard *)


(********************* Main **************************************************)
let () =
  if Array.length Sys.argv <> 3 then begin
    failwith (sprintf "Usage: %s <.xml file> <class_name>" Sys.argv.(0))
  end;

  let filename = Sys.argv.(1)
  and class_name = Sys.argv.(2) in

  try
    let messages = Syntax.read filename class_name in

    let h = stdout in
    
(*    let function_type = "MACROS" in*)
    let function_type = "FUNCTIONS" in

    Printf.fprintf h "/* Automatically generated from %s */\n" filename;
    Printf.fprintf h "/* Please DO NOT EDIT */\n\n";

    Printf.fprintf h "/* %s to send and receive messages of class %s */\n" function_type class_name;
    if (function_type <> "MACROS") then
      Printf.fprintf h "/* Please be sure that _USE_INLINE_TRANSPORT_ is disabled in \"subsystems/datalink/transport2.h\" */\n\n\n"
    else
      Printf.fprintf h "/* Please be sure that _USE_INLINE_TRANSPORT_ is enabled in \"subsystems/datalink/transport2.h\" */\n\n\n";

    Printf.fprintf h "#ifndef _VAR_DOWNLINK_%s_%s_H_\n" function_type class_name;
    Printf.fprintf h "#define _VAR_DOWNLINK_%s_%s_H_\n\n" function_type class_name;
    if class_name = "telemetry" then begin (** FIXME *)
    Printf.fprintf h "#include \"messages_data.h\"\n";
    end else
    Printf.fprintf h "#include \"dl_protocol_data.h\"\n";

    Printf.fprintf h "#include \"subsystems/datalink/transport2.h\"\n";
    Printf.fprintf h "#include \"mcu_periph/device.h\"\n";
    Printf.fprintf h "#include \"mcu_periph/transmit_buffer.h\"\n\n\n";

    Printf.fprintf h "#include <stdio.h> //for printf. Remove after debugging!\n\n\n";

    (** Macros for airborne downlink (sending) *)
    if class_name = "telemetry" then begin (** FIXME *)
    Printf.fprintf h "#ifdef DOWNLINK\n"
    end;
    if (function_type <> "MACROS") then
      List.iter (Gen_onboard.print_downlink_macro h "static inline void" "struct DownlinkTransport *" "tp" "struct device *" "dev" "\n") messages
    else
      List.iter (Gen_onboard.print_downlink_macro h "#define" "" "_trans" "" "_dev" " \\\n") messages;

    if class_name = "telemetry" then begin
      Printf.fprintf h "#else // DOWNLINK\n";
      if (function_type <> "MACROS") then
        List.iter (Gen_onboard.print_null_downlink_macro h "static inline void" "struct DownlinkTransport *" "tp" "struct device *" "dev" "\n") messages
      else
        List.iter (Gen_onboard.print_null_downlink_macro h "#define" "" "_trans" "" "_dev" "\n") messages;
      Printf.fprintf h "#endif // DOWNLINK\n"
    end;

    (** Macros for airborne datalink (receiving) *)
    let check_alignment = class_name <> "telemetry" in
    List.iter (Gen_onboard.print_get_macros h check_alignment) messages;

    Printf.fprintf h "#endif // _VAR_DOWNLINK_%s_%s_H_\n" function_type class_name

  with
      Xml.Error (msg, pos) -> failwith (sprintf "%s:%d : %s\n" filename (Xml.line pos) (Xml.error_msg msg))
