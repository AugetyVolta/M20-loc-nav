import socket

# =============================================
# 1. 协议头构造函数
# =============================================
def build_protocol_header(data_length: int, msg_id: int = 1, asdu_format: int = 0x01) -> bytearray:

    if not (0 <= data_length <= 65535):
        raise ValueError("data_length 必须在 0 ~ 65535 之间")
    if not (0 <= msg_id <= 65535):
        raise ValueError("msg_id 必须在 0 ~ 65535 之间")
    if asdu_format not in [0x00, 0x01]:
        raise ValueError("asdu_format 必须是 0x00（XML）或 0x01（JSON）")

    header = bytearray(16)
    header[0] = 0xeb
    header[1] = 0x91
    header[2] = 0xeb
    header[3] = 0x90
    header[4] = data_length & 0xFF
    header[5] = (data_length >> 8) & 0xFF
    header[6] = msg_id & 0xFF
    header[7] = (msg_id >> 8) & 0xFF
    header[8] = asdu_format  # 0x01 表示 JSON

    # 8~14 字节：预留 7 字节，已经默认是 0，无需设置

    return header

# =============================================
# 2. 基础配置
# =============================================
SERVER_IP = "10.21.31.103"  # 目标服务器 IP
PORT = 30000                # 目标端口

# =============================================
# 3. 构造 JSON 数据
# =============================================
# json_data = """{"PatrolDevice":{"Type":1003,"Command":1,"Time":"2023-01-01 00:00:00","Items":{"Value":0,"MapID":0,"PosX":0.0,"PosY":0.0,"PosZ":0.0,"AngleYaw":0.0,"PointInfo":1,"Gait":12290,"Speed":0,"Manner":0,"ObsMode":1,"NavMode":1}}}"""
json_data = """{"PatrolDevice":{"Type":2,"Command":23,"Time":"2023-01-0100:00:00","Items":{"GaitParam":12290}}}"""
# 转为 UTF-8 bytes，并计算长度（即 ASDU 长度）
asdu_data = json_data.encode('utf-8')
data_length = len(asdu_data)  # 用于协议头中的长度字段

# =============================================
# 4. 构造协议头
#    - 报文ID 示例为 1
#    - ASDU格式为 JSON（0x01）
# =============================================
header = build_protocol_header(
    data_length=data_length,
    msg_id=1,          # 可以设为变量，每次递增
    asdu_format=0x01   # 0x01 表示 JSON
)

# =============================================
# 5. 拼接完整消息：header + asdu_data（JSON）
# =============================================
message = header + asdu_data

# =============================================
# 6. 创建 UDP 套接字并发送
# =============================================
try:
    client_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_address = (SERVER_IP, PORT)

    send_len = client_sock.sendto(message, server_address)
    print(f"消息发送成功！")

except Exception as e:
    print(f"发送失败：{e}")

finally:
    client_sock.close()