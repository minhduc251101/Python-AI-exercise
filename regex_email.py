import re


# 9.1) Kiểm tra địa chỉ IPv4 hợp lệ (x.x.x.x, x trong [0, 255])
def check_ipv4(ip):
    pattern = r'^((25[0-5]|2[0-4]\d|1\d{2}|[1-9]?\d)\.){3}(25[0-5]|2[0-4]\d|1\d{2}|[1-9]?\d)$'
    return bool(re.match(pattern, ip))

print("=== 9.1 IPv4 ===")
print(check_ipv4("192.168.1.1"))   # True
print(check_ipv4("256.0.0.1"))     # False
print(check_ipv4("0.0.0.0"))       # True
print(check_ipv4("999.999.999.999")) # False


# 9.2) Trích xuất tất cả email trong văn bản
def extract_emails(text):
    pattern = r'[a-zA-Z0-9._%+\-]+@[a-zA-Z0-9.\-]+\.[a-zA-Z]{2,}'
    return re.findall(pattern, text)

print("\n9.2 Extract Emails ")
text = "Liên hệ: admin@gmail.com, support@company.org hoặc user.name+tag@sub.domain.vn"
print(extract_emails(text))


# 9.3) Kiểm tra chuỗi khác rỗng, chỉ chứa a-z, A-Z, 0-9
def check_alphanumeric(s):
    pattern = r'^[a-zA-Z0-9]+$'
    return bool(re.match(pattern, s))

print("\n=== 9.3 Alphanumeric ===")
print(check_alphanumeric("Hello123"))  # True
print(check_alphanumeric(""))          # False
print(check_alphanumeric("Hello!"))    # False
print(check_alphanumeric("abc_123"))   # False


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

print("\n=== 9.4 Password Strength ===")
print(check_password("Abc@1234"))     # True
print(check_password("abc@1234"))     # False (thiếu chữ hoa)
print(check_password("Abcdefgh"))     # False (thiếu số và ký tự đặc biệt)
print(check_password("Ab@1"))         # False (quá ngắn)
print(check_password("Abc@12345"))    # True
