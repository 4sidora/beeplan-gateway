# beeplan-gateway

**Сборка, обвязка и связь с ульями и API:** [HARDWARE.md](https://github.com/4sidora/beeplan-docs/blob/main/HARDWARE.md) в репозитории **beeplan-docs** (обновляйте при изменениях прошивки).

Прошивка **BeePlan** для концентратора: приём **ESP‑NOW** от ульевых модулей, упаковка JSON и `POST /v1/telemetry/batch` с заголовком `Authorization: Bearer <ingest_token>`.

В MVP uplink через **Wi‑Fi** (проще отладить). GSM (TinyGSM) добавляется отдельной конфигурацией сборки.

## Настройка

Отредактируйте `include/config.h`: SSID/пароль Wi‑Fi, URL API (например `http://192.168.1.10:8000`), `INGEST_TOKEN` из вывода `python -m beeplan.seed_dev`.

MAC ESP32 концентратора нужно прописать в `beeplan-edge/include/config.h` как `GATEWAY_MAC`.

## Сборка

`pio run -t upload`
