#!/usr/bin/env python3
"""
Park Algoritması - ROS2 Node
-------------------------------
Okur  : /astrid/perception/traffic_sign
Yazar : /astrid/perception/park_line

Mesaja göre gelen park/park_yapılamaz işaretlerini
uzaklığa göre sıralar (en yakın → 1, en uzak → 8)
ve "/astrid/perception/park_line" topiğine yayınlar.

Mesaj formatı örneği:
    "Park, 1"
    "Park_yapılamaz, 3"
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
import json
import re


# ──────────────────────────────────────────────
# Sabitler
# ──────────────────────────────────────────────
SUBSCRIBE_TOPIC = "/astrid/perception/traffic_sign"
PUBLISH_TOPIC   = "/astrid/perception/park_line"
ALLOWED_SIGNS   = {"park", "parkYasak"}
MAX_RANK        = 8          # En uzak işarete verilecek sıra


class ParkSignNode(Node):
    """
    Traffic sign topiğini dinler; Park ve Park_yapılamaz
    işaretlerini filtreler, mesafeye göre sıralar ve yayınlar.
    """

    def __init__(self):
        super().__init__("park_sign_node")

        # Aktif işaret listesi: [{"sign": str, "distance": float}, ...]
        self._signs: list[dict] = []

        # Subscriber
        self.subscription = self.create_subscription(
            String, 
            '/astrid/perception/traffic_sign',
            self._traffic_sign_callback,  # <--- Hata buradaydı, fonksiyonun doğru adını yazdık
            qos_profile_sensor_data  
        )

        # Publisher
        self._pub = self.create_publisher(String, PUBLISH_TOPIC, 10)

        # Periyodik yayın – 1 Hz
        self._timer = self.create_timer(1.0, self._publish_ranked_signs)

        self.get_logger().info(
            f"ParkSignNode başlatıldı.\n"
            f"  Okuma : {SUBSCRIBE_TOPIC}\n"
            f"  Yazma : {PUBLISH_TOPIC}"
        )

    # ──────────────────────────────────────────
    # Callback – gelen mesajı işle
    # ──────────────────────────────────────────
    def _traffic_sign_callback(self, msg: String) -> None:
        """
        Beklenen mesaj formatı (JSON veya virgülle ayrılmış):
            {"sign": "Park", "distance": 12.5}
            veya
            "Park, 12.5"
        """
        data = msg.data.strip()

        sign, distance = self._parse_message(data)

        if sign is None:
            self.get_logger().debug(f"Tanınmayan mesaj formatı: '{data}'")
            return

        if sign.lower() not in ALLOWED_SIGNS:
            self.get_logger().debug(f"İlgisiz işaret atlandı: '{sign}'")
            return

        # Mevcut listede aynı işaret/mesafe çifti yoksa ekle
        entry = {"sign": sign, "distance": distance}
        if entry not in self._signs:
            self._signs.append(entry)
            self.get_logger().info(f"İşaret eklendi → {entry}")
            
        self.get_logger().info(f"GELEN RAW DATA: {msg.data}")
        self.get_logger().info(f"PARSED: sign={sign}, distance={distance}")

    # ──────────────────────────────────────────
    # Yardımcı – mesaj ayrıştırma
    # ──────────────────────────────────────────
    @staticmethod
    def _parse_message(data: str):
        try:
            obj = json.loads(data)

            # Eğer liste ise
            if isinstance(obj, list):
                if not obj:
                    return None, None
                obj = obj[0]

            # Eğer dict ise (beklenen durum)
            if isinstance(obj, dict):
                sign = str(obj.get("sign", "")).strip()
                distance = float(obj.get("distance", 0.0))
                return sign, distance

            # ❗ Eğer string ise (çok önemli eksik case)
            if isinstance(obj, str):
                data = obj  # fallback'e gönder

        except Exception:
            pass
            
        # ── Virgüllü format fallback ──
        pattern = r"^([A-Za-zÇçĞğİıÖöŞşÜü_]+)\s*,\s*([\d.]+)$"
        match = re.match(pattern, data)
        if match:
            sign = match.group(1).strip()
            distance = float(match.group(2).strip())
            return sign, distance

        return None, None

    # ──────────────────────────────────────────
    # Timer – sıralayıp yayınla
    # ──────────────────────────────────────────
    def _publish_ranked_signs(self) -> None:
        if not self._signs:
            return

        ranked = self._rank_signs(self._signs)

        for item in ranked:
            out_msg = String()
            out_msg.data = f"{item['sign']}, {item['rank']}"
            self._pub.publish(out_msg)
            self.get_logger().info(f"Yayınlandı → {out_msg.data}")

        # Her yayın döngüsünden sonra listeyi temizle
        # (Sürekli birikmesini önlemek için; ihtiyaca göre kaldırılabilir)
        self._signs.clear()

    # ──────────────────────────────────────────
    # Sıralama mantığı
    # ──────────────────────────────────────────
    @staticmethod
    def _rank_signs(signs: list[dict]) -> list[dict]:
        """
        İşaretleri mesafeye göre artan sırada sıralar.
        En yakındaki levha 1. park yeri, bir sonraki 2. park yeri olacak
        şekilde (1'den başlayarak ardışık) numaralandırır.
        """
        if not signs:
            return []

        # Uzaklığa (distance) göre küçükten büyüğe sırala
        sorted_signs = sorted(signs, key=lambda x: x["distance"])
        
        ranked_signs = []
        for i, item in enumerate(sorted_signs):
            # Sıra numarası (index 0'dan başladığı için +1 ekliyoruz)
            rank = i + 1
            
            # Görev açıklamasına göre en uzak levha 8. olmalı.
            # Kameradan/Lidar'dan 8'den fazla hatalı levha verisi gelirse (noise) onları yoksay.
            if rank <= MAX_RANK:
                item["rank"] = rank
                ranked_signs.append(item)
            else:
                break # 8'den fazlasına numara atama, döngüden çık.

        return ranked_signs

# ──────────────────────────────────────────────
# Giriş noktası
# ──────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = ParkSignNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node durduruldu (KeyboardInterrupt).")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
