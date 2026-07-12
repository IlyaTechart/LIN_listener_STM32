Бля, а можно просто здесь удалить всё что отностится к github и просто новый рпеоизиторий создать. Можно это сделать через GitHube Desktop ? 



Да, можно. Это самый простой путь, если старая история и старый remote тебе не нужны.

Что удалить локально
В папке проекта нужно убрать только скрытую папку .git — это и есть вся привязка к старому GitHub-репозиторию:

c:\Users\q\Desktop\STM32_Projects\LIN_MASTER_STM32F103\.git
После этого проект станет обычной папкой с файлами, без git-истории и без origin.

В проводнике Windows:

Включи показ скрытых файлов.
Удали папку .git.
Или в PowerShell:

cd "c:\Users\q\Desktop\STM32_Projects\LIN_MASTER_STM32F103"
Remove-Item -Recurse -Force .git
Больше ничего специально «от GitHub» удалять не надо.

Можно ли через GitHub Desktop?
Да, GitHub Desktop для этого отлично подходит.

Вариант 1 — через GitHub Desktop (рекомендую)
Удали .git, как выше.
Сначала создай .gitignore, чтобы не залить Debug/:
Debug/
Release/
*.o
*.elf
*.map
*.su
*.d
*.cyclo
*.list
.settings/
*.launch
Открой GitHub Desktop.
File → Add local repository.
Укажи папку LIN_MASTER_STM32F103.
Desktop скажет, что это не git-репозиторий — нажми create a repository.
Задай имя, например LIN_MASTER_STM32F103.
Проверь список изменений: не должно быть Debug/.
Сделай первый commit.
Publish repository — Desktop сам создаст новый repo на GitHub и запушит.
Вариант 2 — сначала repo на сайте GitHub
На github.com создай пустой репозиторий.
Удали локальную .git.
В GitHub Desktop: Add local repository → create a repository.
Commit.
Repository → Repository settings → Remote или через Publish / Push origin.
Для твоего случая проще именно Publish repository из Desktop.

На что обратить внимание
Не коммить Debug/ — там бинарники и артефакты сборки.
Проверь удалённые HAL-файлы: если проект должен собираться у других, они должны быть в репозитории.
Старый репозиторий LIN_listener_STM32 на GitHub сам не исчезнет — просто перестанет быть связан с этой папкой. Его можно оставить, архивировать или удалить вручную на сайте.
Коротко
Вопрос	Ответ
Можно снести старую привязку?
Да, удалить .git
Можно через GitHub Desktop?
Да, это нормальный способ
Нужен ли старый remote?
Нет
Что обязательно перед publish?
.gitignore и проверка, что нет Debug/
Если хочешь, могу в следующем сообщении дать готовый .gitignore именно под STM32CubeIDE и список папок, которые стоит включить в первый commit.

Напиши Readme об этом проекте 



Сначала соберу детали по проекту — конфигурацию, структуру и логику — чтобы README был точным.

Ниже готовый README.md под текущую версию проекта (LIN_MASTER_STM32F103_V2). Можешь скопировать целиком в файл.


# LIN_MASTER_STM32F103
Прошивка для **STM32F103C8T6**, которая слушает шину **LIN**, принимает кадр с заданным **PID** и управляет выходом **реле** в зависимости от значения байта в данных кадра.
Проект написан **bare-metal** без FreeRTOS: приём LIN выполняется в прерывании USART1, обработка кадров — в главном цикле `main()`.
---
## Назначение
Устройство подключается к LIN-шине как слушатель и реагирует на определённое сообщение:
- если в кадре с нужным PID в заданном байте приходит команда **ON** — включается реле;
- если приходит команда **OFF** — реле выключается.
Типичный сценарий: промежуточный LIN-адаптер между автомобильной/промышленной LIN-шиной и исполнительным устройством (реле, нагрузка и т.п.).
---
## Основные возможности
- Приём LIN-кадров через **USART1** в режиме LIN
- Разбор кадра в прерывании по state machine:
  - Break → Sync (`0x55`) → PID → Data → Checksum
- Проверка контрольной суммы:
  - **Classic** (LIN 1.3)
  - **Enhanced** (LIN 2.x)
- Передача принятых кадров из ISR в `main()` через **кольцевой буфер**
- Управление реле по содержимому конкретного LIN-кадра
- Настройка PID, DLC и байтов команд через макросы в `portable_LIN.h`
---
## Аппаратная часть
| Компонент | Описание |
|-----------|----------|
| MCU | STM32F103C8T6 |
| LIN UART | USART1 |
| LIN TX | PA9 |
| LIN RX | PA10 |
| NSLP (LIN transceiver) | PA11 |
| RELAY | PB3 |
| LED | PB12 |
### Подключение
1. Подключить LIN-трансивер к **PA9/PA10**.
2. Вывести **NSLP** в активное состояние для работы трансивера.
3. Подключить реле к **PB3**.
4. Общая земля MCU, трансивера и LIN-шины обязательна.
---
## Архитектура ПО
```text
LIN bus
   ↓
USART1 (LIN mode, IRQ)
   ↓
USART1_IRQHandler()
   ├─ state machine приёма кадра
   ├─ проверка checksum
   └─ UART_Write_To_Buffer()
           ↓
   ring buffer (ISR → main)
           ↓
main loop
   ├─ UART_Read_From_Buffer()
   ├─ фильтр по PID
   └─ управление RELAY
Разделение ролей
Контекст	Что делает
Прерывание USART1
Принимает байты, собирает кадр, проверяет checksum, кладёт валидный кадр в буфер
main()
Читает кадры из буфера и выполняет прикладную логику
Такой подход убирает race condition: main() не работает напрямую с переменной, которую в этот момент может перезаписать ISR.

Настройка под свою LIN-шину
Все основные параметры находятся в Core/Inc/portable_LIN.h.

Скорость LIN
В Core/Inc/main.h:

#define UART_DAUD_RATE   19200
#define APB2_CLK_MHz     72
PID и полезные данные
В Core/Inc/portable_LIN.h:

#define PID_RECESIVE_FRAME   0x42   // PID кадра, который нужно обрабатывать
#define NUM_BYTE_DATA        0x02   // индекс байта в data[], с которого читается команда
#define CHECK_BYTE_ON        0x03   // значение байта для включения реле
#define CHECK_BYTE_OFF       0x88   // значение байта для выключения реле
Длина кадра (DLC)
#define CALCULATION_DLC_FROM_PID  0
#define USER_DLC_FRAME            8
CALCULATION_DLC_FROM_PID = 0 — длина кадра задаётся вручную через USER_DLC_FRAME
CALCULATION_DLC_FROM_PID = 1 — DLC вычисляется функцией Calculation_DLC()
Размер буфера кадров
#define LIN_FRAME_BUFFER_SIZE  32
Как настроить проект под новую шину
Рекомендуемый порядок:

Подключить LIN-анализатор / sniffer.
Определить скорость шины (обычно 9600 или 19200).
Найти нужный PID и формат данных.
Определить:
номер байта, который меняется;
значения для ON/OFF.
Обновить макросы в portable_LIN.h.
Пересобрать и прошить проект.
Сборка и прошивка
Требования
STM32CubeIDE
Board: STM32F103C8Tx
Проект: LIN_MASTER_STM32F103_V2.ioc
Шаги
Открыть проект в STM32CubeIDE.
Собрать конфигурацию Debug.
Прошить MCU через ST-Link.
Проверить работу на реальной LIN-шине.
Структура проекта
Core/
├── Inc/
│   ├── main.h
│   └── portable_LIN.h      # LIN driver API и конфигурация
├── Src/
│   ├── main.c              # главный цикл и логика реле
│   ├── portable_LIN.c      # инициализация USART1, checksum, ring buffer
│   └── stm32f1xx_it.c      # USART1_IRQHandler, state machine приёма
Drivers/                    # STM32 HAL / CMSIS
LIN_MASTER_STM32F103_V2.ioc # CubeMX-конфигурация
STM32F103C8TX_FLASH.ld      # linker script
Логика работы main()
Упрощённо:

while (1)
{
    if (UART_Read_From_Buffer(&New_Frame))
    {
        if (New_Frame.PID == PID_RECESIVE_FRAME)
        {
            if (New_Frame.data[NUM_BYTE_DATA] == CHECK_BYTE_ON)
                RELAY = ON;
            if (New_Frame.data[NUM_BYTE_DATA] == CHECK_BYTE_OFF)
                RELAY = OFF;
        }
    }
    else
    {
        HAL_Delay(1);
    }
}
Ограничения и замечания
Сейчас в main() не проверяется New_Frame.status == LIN_FRAME_OK. Для максимальной надёжности стоит обрабатывать только кадры с валидной checksum.
При переполнении ring buffer новые кадры теряются, выставляется ful_bufer_flag.
Функции передачи LIN (UART_Transmit) и блокирующего приёма (UART_Reception) отключены макросами и оставлены как вспомогательные.
Папки Debug/ и Release/ не должны попадать в git — только исходники и конфигурация проекта.
Пример .gitignore
Debug/
Release/
*.o
*.elf
*.map
*.su
*.d
*.cyclo
*.list
.settings/
*.launch
Автор
Ilya Volkov

Лицензия
Проект основан на коде STM32Cube / HAL от STMicroelectronics.
Пользовательская логика распространяется по условиям, которые вы укажете в репозитории.

---
Если хочешь, могу сделать ещё две версии:
1. **Короткую** — на 15–20 строк для GitHub.
2. **На английском** — для международного репозитория.
Или в **Agent mode** могу сразу записать этот `README.md` в проект.
