import re
from typing import Any
from enum import Enum

# 9.1) Kiểm tra địa chỉ IPv4 hợp lệ (x.x.x.x, x trong [0, 255])
def check_ipv4(ip) -> bool:
    pattern = r'^((25[0-5]|2[0-4]\d|1\d{2}|[1-9]?\d)\.){3}(25[0-5]|2[0-4]\d|1\d{2}|[1-9]?\d)$'
    return re.match(pattern, ip)


def check_ipv6(ip) -> bool:
    pattern_ = r'^(?:[0-9A-Fa-f]{1,4}:){7}[0-9A-Fa-f]{1,4}$|^::1$|^::$'
    return re.fullmatch(pattern_, ip)

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
    def __init__(self, ip_type: IP_TYPE): # --> ham khoi tao va de dung sau cac bien 
        self.ip_type = ip_type
        self.pattern_v4 = r'^((25[0-5]|2[0-4]\d|1\d{2}|[1-9]?\d)\.){3}(25[0-5]|2[0-4]\d|1\d{2}|[1-9]?\d)$'
        self.pattern_v6 = r'^(?:[0-9A-Fa-f]{1,4}:){7}[0-9A-Fa-f]{1,4}$|^::1$|^::$'

    def check_ip_type_basic(self): 
        return self.ip_type
    
    @property # cho ham chay nhanh hon va khong bi luu them bo nho dem
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
def check_password(pwd:str) -> bool:
    checks = [r"[a-z]", r"[A-Z]", r"[0-9]", r"[!@#$%^&*]"]
    if len(pwd) < 8:
        print("Non-Valid Password")
        return False
        
    if re.search(r"[a-z]", pwd) is None:
        print("Missing lowercase")
    if re.search(r"[A-Z]", pwd) is None:
        print("Missing Uppercase")
    if re.search(r"[0-9]", pwd) is None:
        print("Missing Number")
    if re.search(r"[!@#$%^&*]", pwd) is None:
        print("Missing Special Character")
    
    return all(re.search(p, pwd) is not None for p in checks)

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
    #update check pwd in coding style 
    pwd = "abcxy111112222"
    check_pwd = check_password(pwd) 
    print(check_pwd)
    # checkip = CHECKIP(ip_type=IP_TYPE.V6.value)
    # print(checkip.check_ip_type)
