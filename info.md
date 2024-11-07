## Note

1. Use the overlay file: `nrf52dk_nrf52832.overlay`
2. The board DTS directory `<DBOARD_ROOT>` is located at:
3. Ensure all directory paths are valid in the `CMakeLists.txt` file.
4. For errors related to:
```c
.drv_inst_idx = NRFX_CONCAT_3(NRFX_SPIM, id, _INST_IDX) check the overlay and DTS files used.


