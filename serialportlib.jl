
module Serialportlib

serialportlib = ""
if Sys.iswindows()
    error("Microsoft Windows is not supported by Serialportlib!")
else
    serialportlib = "$(@__DIR__)/libserialportlib.so"
end

const SERIALPORTLIB_MAX_SERIALPORT_IDX = 1024
const DEFAULT_SP_IDX = 0

@enum Serial_Port_Flags begin
    DTR = 0
    RTS = 1
end

function open(portname::String, baudrate; sp_idx=DEFAULT_SP_IDX)::Bool
    @ccall serialportlib.serialport_open(sp_idx::UInt32, portname::Cstring, baudrate::UInt32)::Bool
end

function close(; sp_idx=DEFAULT_SP_IDX)::Bool
    @ccall serialportlib.serialport_close(sp_idx::UInt32)::Bool
end

function is_open(; sp_idx=DEFAULT_SP_IDX)::Bool
    @ccall serialportlib.serialport_is_open(sp_idx::UInt32)::Bool
end

function set_flag(flag::Serial_Port_Flags, value::Bool; sp_idx=DEFAULT_SP_IDX)::Bool
    @ccall serialportlib.serialport_set_flag(sp_idx::UInt32, flag::Cint, value::Bool)::Bool
end

function bytes_available_to_read(; sp_idx=DEFAULT_SP_IDX)::UInt64
    @ccall serialportlib.serialport_bytes_available_to_read(sp_idx::UInt32)::UInt64
end

"Try to read as many as 'out_size' bytes. Returns how many bytes were actually read."
function read_byte(; sp_idx=DEFAULT_SP_IDX)::UInt8
    @ccall serialportlib.serialport_read_byte(sp_idx::UInt32)::UInt8
end

"Try to read as many as 'out_size' bytes. Returns how many bytes were actually read."
function read_bytes(bytes_count; sp_idx=DEFAULT_SP_IDX)::Vector{UInt8}
    bytes = Vector{UInt8}(undef, bytes_count)
    bytes_received = @ccall serialportlib.serialport_read_bytes(sp_idx::UInt32, bytes::Ptr{Cvoid}, bytes_count::UInt64)::UInt64
    resize!(bytes, bytes_received)
    return bytes
end

"""
Comsumes bytes from the serialport until it finds the delimiter. The next bytes read are the ones just after the delimiter.
This is useful for when your data-stream becomes misaligned because a byte got missing etc.
Important: Make sure your bytes-array for the delimiter is contiguous in memory!
"""
function serialport_skip_to_next_delimiter(delimiter::BytesVectorT; sp_idx=DEFAULT_SP_IDX)::Bool where { BytesVectorT <: AbstractVector{UInt8} }
    @ccall serialportlib.serialport_skip_to_next_delimiter(sp_idx::UInt32, delimiter::Ptr{Cvoid}, length(delimiter)::UInt64)::Bool
end

function write_byte(byte; sp_idx=DEFAULT_SP_IDX)::Bool
    @ccall serialportlib.serialport_write_byte(sp_idx::UInt32, byte::UInt8)::Bool
end

"Important: Make sure your bytes-array for the delimiter is contiguous in memory!"
function write_bytes(bytes::BytesVectorT; sp_idx=DEFAULT_SP_IDX)::UInt64 where { BytesVectorT <: AbstractVector{UInt8} }
    @ccall serialportlib.serialport_write_bytes(sp_idx::UInt32, bytes::Ptr{Cvoid}, length(bytes)::UInt64)::UInt64
end

# function read_object(::Type{T}; sp_idx=DEFAULT_SP_IDX) where {T}
#     if !isconcretetype(T)
#         println("SERIALPORTLIB ERROR: The type '$T' is not a concrete type, though it has to be.");
#         return nothing
#     end

#     #NOTE (YUZENI): reinterpret requires a heap allocated vector (so we cannot use StaticArrays)
#     raw_bytes = Vector{UInt8}(undef, sizeof(T))
#     if @ccall serialportlib.serialport_read_object(sp_idx::UInt32, raw_bytes::Ptr{Cvoid}, sizeof(T)::UInt64)::Bool
#         return reinterpret(T, raw_bytes)[1]
#     end
#     return nothing
# end

end # module Serialportlib
