import asyncio
import sys
import argparse
from bleak import BleakClient, BleakScanner

async def list_all_services_and_characteristics(device_name):
    """
    掃描指定名稱的 BLE 裝置，連線後印出其所有的服務和特徵。
    """
    print(f"正在掃描名稱為 '{device_name}' 的裝置...")
    device = await BleakScanner.find_device_by_name(device_name, timeout=10.0)

    if device is None:
        print(f"** 錯誤: 找不到裝置 '{device_name}' **")
        return

    print(f"找到裝置: {device.name} (Address: {device.address})")
    print("正在嘗試連線...")

    try:
        async with BleakClient(device) as client:
            if not client.is_connected:
                print(f"** 錯誤: 無法連線到 {device.address} **")
                return
            
            print(f"\n--- 成功連線到 {device.name} ---")

            # 遍歷所有服務 (Service)
            for service in client.services:
                print(f"\n[服務] UUID: {service.uuid}")
                print(f"  說明: {service.description}")
                
                # 遍歷該服務下的所有特徵 (Characteristic)
                for char in service.characteristics:
                    print(f"    [特徵] UUID: {char.uuid}")
                    print(f"      說明: {char.description}")
                    print(f"      Handle: {char.handle}")
                    
                    # 'properties' 是一個列表，包含 'read', 'write', 'notify' 等字串
                    properties_str = ", ".join(char.properties)
                    print(f"      屬性 (Properties): [{properties_str}]")

                    # (可選) 遍歷該特徵下的所有描述符 (Descriptor)
                    # 描述符通常用來提供額外資訊 (例如 CCCD)
                    for descriptor in char.descriptors:
                        print(f"        [描述符] UUID: {descriptor.uuid}")
                        print(f"          Handle: {descriptor.handle}")
                        
                        # 嘗試讀取描述符的值 (例如，人類可讀的說明)
                        try:
                            value = await client.read_gatt_descriptor(descriptor.handle)
                            # 嘗試解碼為 UTF-8 字串，如果失敗則顯示原始 bytes
                            try:
                                print(f"          值 (Value): '{value.decode('utf-8')}'")
                            except UnicodeDecodeError:
                                print(f"          值 (Value): {value.hex()}")
                        except Exception as e:
                            print(f"          值 (Value): (讀取失敗: {e})")

    except Exception as e:
        print(f"\n連線或探索時發生錯誤: {e}")
    finally:
        print("\n--- 探索完畢，連線已中斷 ---")


# --- 主程式進入點 ---
if __name__ == "__main__":
    
    # 建立參數解析器
    parser = argparse.ArgumentParser(description="BLE 服務探索工具")
    
    # 加入一個必要的參數 'name'
    parser.add_argument(
        "name",
        type=str,
        help="您想要掃描的 BLE 裝置名稱 (例如: 'BlueNRG')"
    )
    
    # 解析參數
    args = parser.parse_args()

    try:
        # 將裝置名稱傳遞給主函數
        asyncio.run(list_all_services_and_characteristics(args.name))
    except KeyboardInterrupt:
        print("\n使用者中斷程式。")
        sys.exit(0)