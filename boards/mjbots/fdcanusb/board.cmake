# SPDX-License-Identifier: LicenseRef-Filics

board_runner_args(openocd "--config=${BOARD_DIR}/support/openocd.cfg")
board_runner_args(jlink "--device=STM32G474CE" "--speed=4000" "--reset-after-load")
board_runner_args(pyocd "--target=stm32g474cetx")
board_runner_args(stm32cubeprogrammer "--port=swd" "--reset-mode=hw")

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/stm32cubeprogrammer.board.cmake)
