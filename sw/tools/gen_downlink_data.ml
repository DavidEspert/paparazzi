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
  (*Counts the FULL size of a message*)
  let rec size_fields_full = fun fields size ->
    match fields with
      (Basic t, _, _)::fields -> size_fields_full fields (size + int_of_string(Syntax.sizeof (Basic t)))
      | (Array (t,varname), _, _)::fields -> (string_of_int(size+1)^"+_"^Syntax.sizeof (Array(t, varname)))
      | [] -> string_of_int(size)

  (*Counts the CONSTANT size of a message*)
  let rec size_fields_cnst = fun fields size ->
    match fields with
      (Basic t, _, _)::fields -> size_fields_cnst fields (size + int_of_string(Syntax.sizeof (Basic t)))
      | (Array (t,varname), _, _)::fields -> (string_of_int(size+1))
      | [] -> string_of_int(size)

  (*Counts the VARIABLE size of a message*)
  let rec size_fields_var = fun fields ->
    match fields with
      (Basic t, _, _)::fields -> size_fields_var fields
      | (Array (t,varname), _, _)::fields -> ("_"^Syntax.sizeof (Array(t, varname)))
      | [] -> "0"

  let size_of_message_full = fun m -> size_fields_full m.fields 0
  let size_of_message_full2 = fun fields -> size_fields_full fields 0
  let size_of_message_cnst = fun m -> size_fields_cnst m.fields 0
  let size_of_message_var = fun m -> size_fields_var m.fields

  let estimated_size_of_message = fun m ->
    try
      List.fold_right
        (fun (t, _, _)  r ->  int_of_string(Syntax.sizeof t)+r)
        m.fields 0
    with
        Failure "int_of_string" -> (-1)


(** PRINT functions *)
  let print_size_of_message = fun h (size,message) ->
    if  size >= 0 then
      fprintf h "%3d : %s\n" size message.name
    else
      fprintf h "  %s : \t%s\n" (size_of_message_full (message)) message.name

  let print_field2 = fun h (t, name, (_f: format option)) ->
    match t with
        Basic _ ->
          fprintf h "  %-8s   %s;\n" (c_type (Syntax.nameof t)) name
      | Array (t, varname) ->
        let _s = Syntax.sizeof (Basic t) in
        fprintf h "  uint8_t    %s;\n" (Syntax.length_name varname);
        fprintf h "  %-8s   *%s;\n" (c_type (Syntax.nameof (Basic t))) name

(*  let print_field = fun h (t, name, (_f: format option)) ->
    match t with
        Basic _ ->
          fprintf h "\t  tp->PutBytes(tp->impl, %s, %s, (void* ) _%s); \n" (dl_type (Syntax.nameof t)) (Syntax.sizeof t) name
      | Array (t, varname) ->
        let _s = Syntax.sizeof (Basic t) in
        fprintf h "\t  tp->PutBytes(tp->impl, DL_TYPE_ARRAY_LENGTH, 1, (void* ) &%s); \n" (Syntax.length_name varname);
        fprintf h "\t  tp->PutBytes(tp->impl, %s, %s * %s, (void* ) _%s); \n" (dl_type (Syntax.nameof (Basic t))) (Syntax.sizeof (Basic t)) (Syntax.length_name varname) name*)

  (* Prints the table of the messages lengths *)
  let print_lengths_array = fun h class_ messages ->
    let sizes = List.map (fun m -> (m.id, size_of_message_full m)) messages in
    let max_id = List.fold_right (fun (id, _m) x -> max x id) sizes min_int in
    let n = max_id + 1 in
    fprintf h "/*#define DOWNLINK_DATA_%s_LENGTHS {" class_;
    for i = 0 to n - 1 do
      if ( (i mod 10) == 0) then
        fprintf h "\n";
      fprintf h "%s,\t" (try "(" ^List.assoc i sizes^")" with Not_found -> "0")
    done;
    fprintf h "}*/\n\n"

  (* Prints messages ordered by name *)
  let print_lengths_ordered = fun h messages ->
    fprintf h "/*\n messages ordered by size (constant messages first):\n";
    let sizes = List.map (fun m -> (estimated_size_of_message m, m)) messages in
    let sizes = List.sort (fun (s1,_) (s2,_) -> compare s2 s1) sizes in
    List.iter (print_size_of_message h) sizes;
    fprintf h "*/\n\n"

  (* Prints parameters in function header *)
  let print_parameter_function_header h = function
  (Array (t, varname), s, _) -> fprintf h "const uint8_t _%s, const %s *_%s" (Syntax.length_name s) (c_type (Syntax.nameof (Basic t))) s
    | (t, s, _) -> fprintf h "const %s *_%s" (c_type (Syntax.nameof t)) s

  let print_parameters_function_header h = function
  [] -> ()
    | f::fields ->
      print_parameter_function_header h f;
      List.iter (fun f -> fprintf h ", "; print_parameter_function_header h f) fields

  (* Prints parameters in function body *)
  let print_parameter_function_body h = function
  (Array (t, varname), s, _) ->
  	fprintf h "  packet.%-20s = _%s;\n" (Syntax.length_name s) (Syntax.length_name s);
  	fprintf h "/*  packet.%-20s = _%s;*/\n" s s
    | (t, s, _) -> fprintf h "  packet.%-20s = *_%s;\n" s s

  let rec fields_count = fun fields size ->
    match fields with
      (Basic t, _, _)::fields -> fields_count fields (size+1)
      | (Array (t,varname), _, _)::fields -> fields_count fields (size+2)
      | [] -> size


  (** Prints data struct *)
  let print_data_struct = fun h {name=s; fields = fields} ->
    let size_full = size_fields_full fields 0 in
    fprintf h "#define DOWNLINK_DATA_%s_LENGTH        (%s)\n" s size_full;
    let size_cnst = size_fields_cnst fields 0 in
    fprintf h "#define DOWNLINK_DATA_%s_LENGTH_CNST   (%s)\n" s size_cnst;
    let size_var = size_fields_var fields in
    fprintf h "#define DOWNLINK_DATA_%s_LENGTH_VAR    (%s)\n" s size_var;
    (*let num_of_fields = fields_count fields 0 in
    fprintf h "//#define MSG_PPRZ_DATA_%s_FIELDS (%d)\n" s num_of_fields;*)
    fprintf h "struct downlink_data_%s {\n" s;
    List.iter (print_field2 h) fields;
    fprintf h "};\n\n"

(** Prints the data_pack function ****************************************************************)
  let rec print_pack_body_var = fun h name fields ->
    match fields with
      (Basic t, _, _)::fields -> print_pack_body_var h name fields
      | (Array (t,varname), s, _)::fields ->
        fprintf h "  memcpy((buff + DOWNLINK_DATA_%s_LENGTH_CNST), _%s, DOWNLINK_DATA_%s_LENGTH_VAR);\n" name s name
      | [] -> ()

  let print_data_pack_function = fun h {name=s; fields = fields} ->
    if List.length fields > 0 then begin
      fprintf h "static inline void downlink_data_%s_pack(uint8_t *buff, " s;
      print_parameters_function_header h fields;
      fprintf h ") {\n\n";
(** DEBUG init
      fprintf h "\tuint8_t msg_len = %s;\n" (size_of_message_full2 fields);
      fprintf h "\_DOWNLINK_DATA_TRACE_(\"\\tdownlink_data_%s_pack (id %%d): data_len = %%u\\n\", DL_%s_ID, msg_len);\n\n" s s;
    DEBUG end *)
      fprintf h "  struct downlink_data_%s     packet;\n\n" s;
      List.iter (print_parameter_function_body h) fields;
      fprintf h "\n";
      fprintf h "  memcpy(buff, &packet, DOWNLINK_DATA_%s_LENGTH_CNST);\n" s;
      print_pack_body_var h s fields;
      fprintf h "}\n\n"
    end else
      fprintf h "/*static inline void downlink_data_%s_pack(uint8_t *buff) {}*/\n\n" s

(** Prints the data_encode function **************************************************************)
  let rec print_encode_body_var  = fun h name fields ->
    match fields with
      (Basic t, _, _)::fields -> print_encode_body_var  h name fields
      | (Array (t,varname), s, _)::fields ->
        fprintf h "  memcpy((buff + DOWNLINK_DATA_%s_LENGTH_CNST), packet->%s, ((packet->%s)*%s));\n" name s (Syntax.length_name s) (Syntax.sizeof (Basic t))
      | [] -> ()

  let print_data_encode_function = fun h {name=s; fields = fields} ->
    if List.length fields > 0 then begin
      fprintf h "static inline void downlink_data_%s_encode(uint8_t *buff, struct downlink_data_%s *packet) {\n\n" s s;
      fprintf h "  memcpy(buff, packet, DOWNLINK_DATA_%s_LENGTH_CNST);\n" s;
      print_encode_body_var h s fields;
      fprintf h "}\n\n"
    end else
      fprintf h "/*static inline void downlink_data_%s_encode(uint8_t *buff, struct downlink_data_%s *packet) {}*/\n\n" s s

(** Prints the function to deserialize a message *************************************************)
  let print_parameter_decode_header  = fun h msg_name field ->
    match field with
      (Array (t, varname), s, _) ->
        fprintf h ", %s *_%s" (c_type (Syntax.nameof (Basic t))) s
      | (t, s, _) -> fprintf h ""

  let print_parameter_decode_body2  = fun h msg_name field ->
    match field with
      (Array (t, varname), s, _) ->
        fprintf h "  uint8_t _%-20s = DL_%s_%s_length(buff);\n" (Syntax.length_name s) msg_name s;
        fprintf h "  void *%-20s    = DL_%s_%s(buff);\n" (s^"_p") msg_name s
(*        fprintf h "  %s *%s = DL_%s_%s(buff);\n" (c_type (Syntax.nameof (Basic t))) s msg_name s*)
      | (t, s, _) -> fprintf h ""

  let print_parameter_decode_body  = fun h msg_name field ->
    match field with
      (Array (t, varname), s, _) ->
        fprintf h "  packet->%-20s = _%s;\n" (Syntax.length_name s) (Syntax.length_name s);
        fprintf h "  packet->%-20s = _%s;\n" s s;
        fprintf h "  memcpy(_%s, %s, DOWNLINK_DATA_%s_LENGTH_VAR);\n" s (s^"_p") msg_name
      | (t, s, _) -> fprintf h "  packet->%-20s = DL_%s_%s(buff);\n" s msg_name s

  let print_data_decode_function = fun h {name=s; fields = fields} ->
    if List.length fields > 0 then begin
      fprintf h "static inline void downlink_data_%s_decode(uint8_t *buff, struct downlink_data_%s *packet" s s;
      List.iter (print_parameter_decode_header h s) fields;
      fprintf h ") {\n";
      List.iter (print_parameter_decode_body2 h s) fields;
      fprintf h "\n";
      List.iter (print_parameter_decode_body h s) fields;
      fprintf h "}\n\n"
    end else
      fprintf h "/*static inline void downlink_data_%s_decode(uint8_t *buff, struct downlink_data_%s *packet) {}*/\n\n" s s

  (** Prints the messages ids *)
  let print_enum = fun h class_ messages ->
    List.iter (fun m ->
      if m.id < 0 || m.id > 255 then begin
        fprintf stderr "Error: message %s has id %d but should be between 0 and 255\n" m.name m.id; exit 1;
      end
      else fprintf h "#define DL_%s_ID  %d\n" m.name m.id
    ) messages;
    fprintf h "#define DL_MSG_%s_NB %d\n\n" class_ (List.length messages)


(** Prints the macro to get access to the fields of a received message ***************************)
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

          fprintf h "#define DL_%s_%s(_payload) ((void*)(_payload+%d))\n" msg_name field_name !offset;
(*          fprintf h "#define DL_%s_%s(_payload) (( %s* )(_payload+%d))\n" msg_name field_name pprz_type.Pprz.inttype !offset;*)
          offset := -1 (** Mark for no more fields *)
    in

    (** Do it for all the fields of the message *)
    List.iter parse_field message.fields;
    fprintf h "\n"

  let print_message_functions = fun h check_alignment message ->
    fprintf h "\n\n// %s ----------------------------------------------------------\n" message.name;
    print_data_struct h message;
    fprintf h "// --- serialize data ---\n";
    print_data_pack_function h message;
    print_data_encode_function h message;
    fprintf h "// --- deserialize data ---\n";
    print_get_macros h check_alignment message;
    print_data_decode_function h message

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

    Printf.fprintf h "/* Automatically generated from %s */\n" filename;
    Printf.fprintf h "/* Please DO NOT EDIT */\n";
    Printf.fprintf h "/* Macros to send and receive messages of class %s */\n\n\n" class_name;
    Printf.fprintf h "#ifndef _DOWNLINK_DATA_%s_H_\n" class_name;
    Printf.fprintf h "#define _DOWNLINK_DATA_%s_H_\n\n\n" class_name;
    Printf.fprintf h "#include <stdint.h>\n";
    Printf.fprintf h "#include <string.h> //required for memcpy\n\n\n";

    Printf.fprintf h "#ifdef __IEEE_BIG_ENDIAN /* From machine/ieeefp.h */\n#define Swap32IfBigEndian(_u) { _u = (_u << 32) | (_u >> 32); }\n#else\n#define Swap32IfBigEndian(_) {}\n#endif\n\n";

    (** Data structs declaration *)
    (*Printf.fprintf h "#ifdef DOWNLINK\n";*)
    Gen_onboard.print_lengths_ordered h messages;
(*    List.iter (Gen_onboard.print_data_pack_function h) messages;*)

    (** Macros for airborne datalink (receiving) *)
    let check_alignment = class_name <> "telemetry" in
(*    List.iter (Gen_onboard.print_get_macros h check_alignment) messages;*)
    List.iter (Gen_onboard.print_message_functions h check_alignment) messages;

    Printf.fprintf h "#endif // _DOWNLINK_DATA_%s_H_\n" class_name

  with
      Xml.Error (msg, pos) -> failwith (sprintf "%s:%d : %s\n" filename (Xml.line pos) (Xml.error_msg msg))
