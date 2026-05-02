#!/usr/bin/env python3
"""
Park Algoritması - ROS2 Node
-------------------------------
Görev Açıklaması'na uygun olarak:
Okur  : /astrid/perception/traffic_sign
Yazar : /astrid/perception/park_line

Gelen park/parkYasak işaretlerini uzaklığa göre sıralar
(en yakın → 1, en uzak → 8) ve yayınlar.

Sıralama mantığı:
  - İlk işaret geldiğinde 10 saniyelik geri sayım başlar.
  - 10 saniye dolduğunda o ana kadar biriken tüm işaretler
    mesafeye göre sıralanıp yayınlanır.
  - Ardışık iki işaret arasındaki mesafe farkı GAP_THRESHOLD'u
    aşarsa sıra numarası 1 atlanır (boşluk bırakılır).
  - Yayın sonrası liste ve sayaç sıfırlanır, yeni döngü bekler.
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
SUBSCRIBE_TOPIC  = "/astrid/perception/traffic_sign"
PUBLISH_TOPIC    = "/astrid/perception/park_line"

ALLOWED_SIGNS    = {"park", "parkyasak"}
MAX_RANK         = 8
WINDOW_SECONDS   = 10.0   # İlk işaretten itibaren bekleme süresi

# Ardışık iki işaret arasındaki mesafe farkı bu değeri aşarsa
# sıra numarası 1 atlanır (boşluk = farklı bölge/sokak anlamına gelir)
GAP_THRESHOLD    = 3.0    # metre — ihtiyaca göre ayarlanabilir


class ParkSignNode(Node):
    def __init__(self):
        super().__init__("park_sign_node")

        self._signs: list[dict] = []
        self._window_active = False

        self.subscription = self.create_subscription(
            String,
            SUBSCRIBE_TOPIC,
            self._traffic_sign_callback,
            qos_profile_sensor_data
        )

        self._pub = self.create_publisher(String, PUBLISH_TOPIC, 10)

        self._window_timer = self.create_timer(
            WINDOW_SECONDS, self._publish_ranked_signs
        )
        self._window_timer.cancel()

        self.get_logger().info(
            f"ParkSignNode başlatıldı.\n"
            f"  Okuma     : {SUBSCRIBE_TOPIC}\n"
            f"  Yazma     : {PUBLISH_TOPIC}\n"
            f"  Pencere   : {WINDOW_SECONDS} sn\n"
            f"  Boşluk eşiği: {GAP_THRESHOLD} m"
        )

    # ──────────────────────────────────────────
    # Callback – gelen mesajı işle
    # ──────────────────────────────────────────
    def _traffic_sign_callback(self, msg: String) -> None:
        data = msg.data.strip()
        sign, distance = self._parse_message(data)

        if sign is None:
            return

        if sign.lower() not in ALLOWED_SIGNS:
            self.get_logger().debug(f"İlgisiz işaret atlandı: '{sign}'")
            return

        already_exists = any(
            s["sign"].lower() == sign.lower() and s["distance"] == distance
            for s in self._signs
        )
        if not already_exists:
            self._signs.append({"sign": sign, "distance": distance})

        self.get_logger().info(f"PARSED: sign={sign}, distance={distance}")

        if not self._window_active:
            self._window_active = True
            self._window_timer.reset()
            self.get_logger().info(
                f"⏱  İlk işaret alındı. {WINDOW_SECONDS} sn sonra yayınlanacak."
            )

    # ──────────────────────────────────────────
    # Yardımcı – mesaj ayrıştırma
    # ──────────────────────────────────────────
    @staticmethod
    def _parse_message(data: str):
        try:
            obj = json.loads(data)

            if isinstance(obj, list):
                if not obj:
                    return None, None
                inner = obj[0]
                if isinstance(inner, list) and len(inner) >= 2:
                    return str(inner[0]).strip(), float(inner[1])
                elif isinstance(inner, dict):
                    return str(inner.get("sign", "")).strip(), float(inner.get("distance", 0.0))

            elif isinstance(obj, dict):
                return str(obj.get("sign", "")).strip(), float(obj.get("distance", 0.0))

            elif isinstance(obj, str):
                data = obj

        except Exception:
            pass

        pattern = r"^([A-Za-zÇçĞğİıÖöŞşÜü_]+)\s*,\s*([\d.]+)$"
        match = re.match(pattern, data)
        if match:
            return match.group(1).strip(), float(match.group(2).strip())

        return None, None

    # ──────────────────────────────────────────
    # Timer callback – 10 sn doldu, sıralayıp yayınla
    # ──────────────────────────────────────────
    def _publish_ranked_signs(self) -> None:
        self._window_timer.cancel()
        self._window_active = False

        if not self._signs:
            self.get_logger().info("10 sn doldu ama işaret listesi boş, atlanıyor.")
            return

        self.get_logger().info(
            f"10 sn doldu. {len(self._signs)} işaret sıralanıp yayınlanıyor..."
        )

        ranked = self._rank_signs(self._signs)

        for item in ranked:
            out_msg = String()

            if item['sign'].lower() == "parkyasak":
                sign_formatted = "parkYasak"
            else:
                sign_formatted = "Park"

            out_msg.data = f"{sign_formatted}, {item['rank']}"
            self._pub.publish(out_msg)
            self.get_logger().info(f"Yayınlandı → {out_msg.data}")

        self._signs.clear()

    # ──────────────────────────────────────────
    # Sıralama mantığı  ← sadece burası değişti
    # ──────────────────────────────────────────
    @staticmethod
    def _rank_signs(signs: list[dict]) -> list[dict]:
        if not signs:
            return []

        sorted_signs = sorted(signs, key=lambda x: x["distance"])

        ranked_signs = []
        rank = 1  # Mevcut sıra numarası

        for i, item in enumerate(sorted_signs):
            if rank > MAX_RANK:
                break

            ranked_signs.append({
                "sign": item["sign"],
                "distance": item["distance"],
                "rank": rank
            })

            # Sonraki eleman varsa mesafe farkına bak
            if i + 1 < len(sorted_signs):
                gap = sorted_signs[i + 1]["distance"] - item["distance"]
                if gap > GAP_THRESHOLD:
                    # Büyük boşluk → sıra numarasını 1 fazla artır (atla)
                    rank += 2
                else:
                    rank += 1

        return ranked_signs


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
