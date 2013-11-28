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

  (** Prints the messages ids *)
  let print_message_id = fun h message ->
    if message.id < 0 || message.id > 255 then begin
      fprintf stderr "Error: message %s has id %d but should be between 0 and 255\n" message.name message.id; exit 1;
    end
    else fprintf h "#define DL_%s_ID        %d\n" message.name message.id


  (** Prints downlink macro *)
  let print_downlink_macro = fun h function_type message ->
    let s = message.name in
    let fields = message.fields in
    let data_len1 = ("DOWNLINK_DATA_"^s^"_LENGTH" ) in
    let data_len2 = "("^(size_of_message message)^")" in

    fprintf h "// %s ----------------------------------\n" s;
    print_message_id h message;
    fprintf h "#define DL_%s_PRIORITY  0\n\n"s;

    let f_type = ref "" in
    let trans_type = ref "" in
    let trans_name = ref "" in
    let dev_type = ref "" in
    let dev_name = ref "" in
    let eol = ref "" in
    
    if (function_type <> "MACROS") then begin
      f_type := "static inline void";
      trans_type:= "struct transport2 *";
      trans_name := "tp";
      dev_type := "struct device *";
      dev_name := "dev";
      eol:= "\n";
    end else begin
      f_type := "#define";
      (* trans_type:= ""; *)
      trans_name := "_trans";
      (* dev_type := ""; *)
      dev_name := "_dev";
      eol:= " \\\n";
    end;

    if List.length fields > 0 then begin
      fprintf h "%s DOWNLINK_SEND_%s(%s%s, %s%s, " !f_type s !trans_type !trans_name !dev_type !dev_name;
      if (function_type <> "MACROS") then
        print_parameters_function_header h fields
      else
        print_parameters_macro_header h fields;
      fprintf h "){%s" !eol
    end else
      fprintf h "%s DOWNLINK_SEND_%s(%s%s, %s%s){%s" !f_type s !trans_type !trans_name !dev_type !dev_name !eol;

    fprintf h "  uint8_t dev_slot;%s" !eol;
    fprintf h "  uint8_t buff_slot;%s" !eol;
    if (function_type <> "MACROS") then begin
      fprintf h "  uint8_t ta_len =    %s->api.transaction_len;%s" !dev_name !eol;
      fprintf h "  uint8_t tp_hd_len = %s->api.header_len;%s" !trans_name !eol;
      fprintf h "  uint8_t tp_tl_len = %s->api.tail_len;%s" !trans_name !eol;
    end else begin
      fprintf h "  uint8_t ta_len =    msg_join(%s, _transaction_len());%s" !dev_name !eol;
      fprintf h "  uint8_t tp_hd_len = msg_join(%s, _header_len);%s" !trans_name !eol;
      fprintf h "  uint8_t tp_tl_len = msg_join(%s, _tail_len);%s" !trans_name !eol;
    end;
    fprintf h "%s" !eol;

    fprintf h "  _DOWNLINK_SEND_TRACE_(\"\\nDOWNLINK_SEND_%s:\\n\");%s" s !eol;
    fprintf h "%s" !eol;

    fprintf h "  /* 1.- try to get a device's 'transaction' slot */%s" !eol;
    if (function_type <> "MACROS") then
      fprintf h "  if(%s->api.check_free_space(%s->periph, &dev_slot)){%s" !dev_name !dev_name !eol
    else
      fprintf h "  if( msg_join(%s, _check_free_space(&dev_slot)) ){%s" !dev_name !eol;

    fprintf h "    /* 2.- try to get a slot in dynamic buffer */%s" !eol;
    if (function_type <> "MACROS") then
      fprintf h "    if(dynamic_buffer_check_free_space(&dynamic_buff, (ta_len + tp_hd_len + MSG_HD_LEN + %s + tp_tl_len), &buff_slot)){%s" data_len1 !eol
    else
      fprintf h "    if(dynamic_buffer_check_free_space(&dynamic_buff, (ta_len + tp_hd_len + MSG_HD_LEN + %s + tp_tl_len), &buff_slot)){%s" data_len2 !eol;

    fprintf h "      /* 3.- get buffer pointer */%s" !eol;
    fprintf h "      uint8_t *buff = dynamic_buffer_get_slot_pointer(&dynamic_buff, buff_slot);%s" !eol;
    fprintf h "%s" !eol;

    fprintf h "      /* SET TRANSACTION: CONTAINS MESSAGE POINTER, LENGTH AND CALLBACK */%s" !eol;
    fprintf h "      /* 4.- set transaction in buffer */%s" !eol;
    if (function_type <> "MACROS") then
      fprintf h "      %s->api.transaction_pack(buff, (buff + ta_len), (tp_hd_len + MSG_HD_LEN + %s + tp_tl_len), &message_callback);%s" !dev_name data_len1 !eol
    else
      fprintf h "      msg_join(%s, _transaction_pack(buff, (buff + ta_len), (tp_hd_len + MSG_HD_LEN + %s + tp_tl_len), &message_callback));%s" !dev_name data_len2 !eol;
    fprintf h "%s" !eol;

    fprintf h "      /* SET MESSAGE: CONTAINS TRANSPORT HEADER, MESSAGE HEADER, MESSAGE DATA AND TRANSPORT TAIL */%s" !eol;
    fprintf h "      /* 5.- set transport HEADER in buffer (it depends on transport layer) */%s" !eol;
    if (function_type <> "MACROS") then
      fprintf h "      %s->api.header(buff + ta_len, %s + MSG_HD_LEN);%s" !trans_name data_len1 !eol
    else
      fprintf h "      msg_join(%s, _header(buff + ta_len, %s + MSG_HD_LEN));%s" !trans_name data_len2 !eol;

    fprintf h "      /* 6.- set message HEADER in buffer */%s" !eol;
    fprintf h "      msg_hd.msg_id = DL_%s_ID;%s" s !eol;
    fprintf h "      memcpy((buff + ta_len + tp_hd_len), &msg_hd, MSG_HD_LEN);%s" !eol;

    fprintf h "      /* 7.- set message DATA in buffer */%s" !eol;
    if List.length fields > 0 then begin
      fprintf h "      downlink_data_%s_pack((buff + ta_len + tp_hd_len + MSG_HD_LEN), " s;
      print_parameters_macro_header h fields;
      fprintf h ");%s" !eol;
    end;

    fprintf h "      /* 8.- set transport TAIL in buffer (it depends on transport layer) */%s" !eol;
    if (function_type <> "MACROS") then
      fprintf h "      %s->api.tail(buff + ta_len, %s + MSG_HD_LEN);%s" !trans_name data_len1 !eol
    else
      fprintf h "      msg_join(%s, _tail(buff + ta_len, %s + MSG_HD_LEN));%s" !trans_name data_len2 !eol;
    fprintf h "%s" !eol;

    fprintf h "      /* SUMMIT TRANSACTION */%s" !eol;
    fprintf h "      /* 9.- send message */%s" !eol;
    if (function_type <> "MACROS") then
      fprintf h "      %s->api.transaction_summit(%s->periph, dev_slot, buff, DL_%s_PRIORITY);%s" !dev_name !dev_name s !eol
    else
      fprintf h "      msg_join(%s, _sendMessage(dev_slot, buff, DL_%s_PRIORITY));%s" !dev_name s !eol;
    fprintf h "    }%s" !eol;

    fprintf h "    else {%s" !eol;
    fprintf h "      /* 10.- release device's slot */%s" !eol;
    if (function_type <> "MACROS") then
      fprintf h "      %s->api.free_space(%s->periph, dev_slot);%s" !dev_name !dev_name !eol
    else
      fprintf h "      msg_join(%s, _free_space(dev_slot));%s" !dev_name !eol;
    fprintf h "    }%s" !eol;
    fprintf h "  }%s" !eol;
    fprintf h "}\n\n"





  let print_null_downlink_macro = fun h function_type {name=s; fields = fields} ->
    let f_type = ref "" in
    let trans_type = ref "" in
    let trans_name = ref "" in
    let dev_type = ref "" in
    let dev_name = ref "" in
    
    if (function_type <> "MACROS") then begin
      f_type := "static inline void";
      trans_type:= "struct transport2 *";
      trans_name := "tp";
      dev_type := "struct device *";
      dev_name := "dev";
    end else begin
      f_type := "#define";
      (* trans_type:= ""; *)
      trans_name := "_trans";
      (* dev_type := ""; *)
      dev_name := "_dev";
    end;

    if List.length fields > 0 then begin
      fprintf h "%s DOWNLINK_SEND_%s(%s%s, %s%s, " !f_type s !trans_type !trans_name !dev_type !dev_name;
      if (function_type <> "MACROS") then
        print_parameters_function_header h fields
      else
        print_parameters_macro_header h fields;
      fprintf h ") {}\n\n"
    end else
      fprintf h "%s DOWNLINK_SEND_%s(%s%s, %s%s) {}\n\n" !f_type s !trans_type !trans_name !dev_type !dev_name

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

    Printf.fprintf h "#ifndef _VAR_DOWNLINK_%s_%s_H_\n" function_type class_name;
    Printf.fprintf h "#define _VAR_DOWNLINK_%s_%s_H_\n\n" function_type class_name;

    (** Macros for airborne downlink (sending) *)
    if class_name = "telemetry" then begin (** FIXME *)
    Printf.fprintf h "#ifdef DOWNLINK\n\n"
    end;

    Printf.fprintf h "#include \"subsystems/datalink/messages_header.h\"\n";
    if class_name = "telemetry" then begin (** FIXME *)
    Printf.fprintf h "#include \"messages_data.h\"\n";
    end else
    Printf.fprintf h "#include \"dl_protocol_data.h\"\n";
    Printf.fprintf h "#include \"subsystems/datalink/transport2.h\"\n";
    Printf.fprintf h "#include \"subsystems/datalink/device.h\"\n\n\n";

    if (function_type <> "FUNCTIONS") then
      Printf.fprintf h "#define __msg_join(_y, _x) _y##_x\n#define _msg_join(_y, _x) __msg_join(_y, _x)\n#define msg_join(_chan, _fun) _msg_join(_chan, _fun)\n\n\n";

    List.iter (Gen_onboard.print_downlink_macro h function_type) messages;

    if class_name = "telemetry" then begin
      Printf.fprintf h "#else // DOWNLINK\n";
      List.iter (Gen_onboard.print_null_downlink_macro h function_type) messages;
      Printf.fprintf h "#endif // DOWNLINK\n"
    end;

    (** Macros for airborne datalink (receiving) *)
(*    let check_alignment = class_name <> "telemetry" in
    List.iter (Gen_onboard.print_get_macros h check_alignment) messages;*)

    Printf.fprintf h "#endif // _VAR_DOWNLINK_%s_%s_H_\n" function_type class_name

  with
      Xml.Error (msg, pos) -> failwith (sprintf "%s:%d : %s\n" filename (Xml.line pos) (Xml.error_msg msg))
