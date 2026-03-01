import re
from typing import Any
from enum import Enum

# 9.1) Kiểm tra địa chỉ IPv4 hợp lệ (x.x.x.x, x trong [0, 255])
def check_ipv4(ip) -> bool:
    pattern = r'^((25[0-5]|2[0-4]\d|1\d{2}|[1-9]?\d)\.){3}(25[0-5]|2[0-4]\d|1\d{2}|[1-9]?\d)$'
    return re.match(pattern, ip)

class IP_TYPE(Enum):
    V4 = "v4"
    V6 = "v6"


## basic pattern -> struct
def check_ip(ip: str, ip_type: IP_TYPE = IP_TYPE) -> bool:
    if isinstance(ip, int):
        raise ValueError("must be define the type")
    if ip_type == IP_TYPE.V4.value:
        print("USING V4")
        pattern = r'^((25[0-5]|2[0-4]\d|1\d{2}|[1-9]?\d)\.){3}(25[0-5]|2[0-4]\d|1\d{2}|[1-9]?\d)$'
        if re.match(pattern, ip):
            print("This is a ipv4")
            return True # early return
        print("this isn't a ipv4")
        return False
    elif ip_type == IP_TYPE.V6.value:
        print("USING V6")
        #TODO: implement define the pattern of IPv6
        return True
    print("code sai")
    return False

class CHECKIP: # --> 
    def __init__(self, ip_type: IP_TYPE): # --> 
        self.ip_type = ip_type
        self.pattern_v4 = r'^((25[0-5]|2[0-4]\d|1\d{2}|[1-9]?\d)\.){3}(25[0-5]|2[0-4]\d|1\d{2}|[1-9]?\d)$'

    def check_ip_type_basic(self): 
        return self.ip_type
    
    @property
    def check_ip_type(self): 
        return self.ip_type
    # pattern = r'^((25[0-5]|2[0-4]\d|1\d{2}|[1-9]?\d)\.){3}(25[0-5]|2[0-4]\d|1\d{2}|[1-9]?\d)$'
    # return bool(re.match(pattern, ip))

# print("=== 9.1 IPv4 ===")
# print(check_ipv4("192.168.1.1"))   # True
# print(check_ipv4("256.0.0.1"))     # False
# print(check_ipv4("0.0.0.0"))       # True
# print(check_ipv4("999.999.999.999")) # False


# 9.2) Trích xuất tất cả email trong văn bản
def extract_emails(text):
    pattern = r'[a-zA-Z0-9._%+\-]+@[a-zA-Z0-9.\-]+\.[a-zA-Z]{2,}'
    return re.findall(pattern, text)

# print("\n9.2 Extract Emails ")
# text = "Liên hệ: admin@gmail.com, support@company.org hoặc user.name+tag@sub.domain.vn"
# print(extract_emails(text))


# 9.3) Kiểm tra chuỗi khác rỗng, chỉ chứa a-z, A-Z, 0-9
def check_alphanumeric(s):
    pattern = r'^[a-zA-Z0-9]+$'
    return bool(re.match(pattern, s))

# print("\n=== 9.3 Alphanumeric ===")
# print(check_alphanumeric("Hello123"))  # True
# print(check_alphanumeric(""))          # False
# print(check_alphanumeric("Hello!"))    # False
# print(check_alphanumeric("abc_123"))   # False


# 9.4) Kiểm tra độ mạnh mật khẩu
#   - Độ dài >= 8
#   - Ít nhất 1 chữ thường
#   - Ít nhất 1 chữ hoa
#   - Ít nhất 1 chữ số
#   - Ít nhất 1 ký tự đặc biệt [!@#$%^&*]
def check_password(pwd):
    if len(pwd) < 8:
        return False
    checks = [
        r'[a-z]',          # chữ thường
        r'[A-Z]',           # chữ hoa
        r'[0-9]',           # chữ số
        r'[!@#$%^&*]',      # ký tự đặc biệt
    ]
    return all(re.search(p, pwd) for p in checks)

# print("\n=== 9.4 Password Strength ===")
# print(check_password("Abc@1234"))     # True
# print(check_password("abc@1234"))     # False (thiếu chữ hoa)
# print(check_password("Abcdefgh"))     # False (thiếu số và ký tự đặc biệt)
# print(check_password("Ab@1"))         # False (quá ngắn)
# print(check_password("Abc@12345"))    # True


if __name__ == "__main__":
    input_ip = "192.168.1.1.1"
    # print(check_ipv4(input_ip)) ## true
    # print('*'*40)
    # print(check_ip(input_ip, ip_type="v4")) ## true
    checkip = CHECKIP(ip_type=IP_TYPE.V4.value)
    print(checkip.check_ip_type)
