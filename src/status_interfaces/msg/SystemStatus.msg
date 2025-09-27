# 时间戳字段：记录数据产生的精确时间
# 类型为ROS 2标准时间类型builtin_interfaces/Time
builtin_interfaces/Time stamp 

# 主机名称字段：标识数据来源的设备名称
string host_name 

# CPU使用率：当前系统CPU的总体占用百分比（0-100）
float32 cpu_percent

# 内存使用率：已使用内存占总内存的百分比（0-100）
float32 memory_percent

# 总内存容量：系统物理内存总大小（通常单位为GB）
float32 memory_total

# 可用内存容量：当前可分配的内存大小（通常单位为GB）
float32 memory_available

# 网络发送量：累计发送的网络数据总量（单位为字节）
float64 net_sent

# 网络接收量：累计接收的网络数据总量（单位为字节）
float64 net_recv