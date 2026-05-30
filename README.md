# beeplan-gateway

**Сборка, обвязка и связь с ульями и API:** [HARDWARE.md](https://github.com/4sidora/beeplan-docs/blob/main/HARDWARE.md) в репозитории **beeplan-docs** (обновляйте при изменениях прошивки).

Прошивка **BeePlan** для концентратора: приём **ESP‑NOW** от ульевых модулей, упаковка JSON и `POST /v1/telemetry/batch` с заголовком `Authorization: Bearer <ingest_token>`.

**Рекомендуемая установка:** веб-мастер в [beeplan-web](https://github.com/4sidora/beeplan-web) (`/install/gateway`). После boot шлёт `POST /v1/concentrators/heartbeat` с MAC.

В MVP uplink через **Wi‑Fi** (проще отладить). GSM (TinyGSM) добавляется отдельной конфигурацией сборки.

## Настройка (ручная)

Скопируйте `include/config.h.example` → `include/config.h`: SSID/пароль Wi‑Fi, URL API, `INGEST_TOKEN`.

## Сборка

`pio run -t upload`
