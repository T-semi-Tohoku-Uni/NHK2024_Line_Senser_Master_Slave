# NHK2024_Line_Senser_Master_Slave

## 各種パラメータ
### SPI1 Mode and Configuration
- Mode: Full-Duplex Master
- NSS Signal: Disable
- Baund Rate(クロックの周波数?): 664.062KBits/s (1.2MBits/sを超えないように)
- Clock Polarity: High (アイドル時の状態)
- Clock Phase: 2 (High -> Low -> Highで取得)

### GPIOの初期設定
High

## 実装すること
- SPI通信でデータの取得
- ローパスフィルターかけてノイズとる
- 横ずれ, 角度ずれの検出

## 参考資料
- `MCP3002`と`Arduino`で遊んでいるサイト
https://www.denshi.club/cookbook/adda/adc/a-d110spi-mcp3002.html

- `STM32`を使ってるやつ \
https://moons.link/post-356/ \
https://rt-net.jp/mobility/archives/19984
