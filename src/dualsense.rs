//! Sony DualSense (PS5) controller Bluetooth HID host.
//!
//! Uses ESP-IDF's Bluetooth Classic HID Host API to:
//!   1. Scan for and connect to a DualSense controller
//!   2. Parse HID input reports into structured gamepad state
//!
//! The DualSense uses Bluetooth Classic (BR/EDR) HID profile.
//! Its input report (report ID 0x01) layout for buttons/sticks:
//!
//! Byte offsets within the HID input report (after report ID):
//!   0: Left stick X  (0=left, 128=center, 255=right)
//!   1: Left stick Y  (0=up, 128=center, 255=down)
//!   2: Right stick X
//!   3: Right stick Y
//!   4: L2 trigger (0..255)
//!   5: R2 trigger (0..255)
//!   7: Buttons bitfield byte 0 (D-pad in lower nibble, square/cross/circle/triangle in upper)
//!   8: Buttons bitfield byte 1 (L1/R1/L2btn/R2btn/share/options/L3/R3)
//!   9: Buttons bitfield byte 2 (PS/touchpad/mute)

use esp_idf_sys as sys;
use log::{info, warn, error};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Mutex;

/// Parsed DualSense gamepad state.
#[derive(Debug, Clone, Copy, Default)]
pub struct GamepadState {
    /// Left stick X: -1.0 (left) to 1.0 (right)
    pub left_x: f32,
    /// Left stick Y: -1.0 (down/back) to 1.0 (up/forward)
    pub left_y: f32,
    /// Right stick X: -1.0 (left) to 1.0 (right)
    pub right_x: f32,
    /// Right stick Y: -1.0 (down) to 1.0 (up)
    pub right_y: f32,
    /// L2 trigger: 0.0 to 1.0
    pub l2: f32,
    /// R2 trigger: 0.0 to 1.0
    pub r2: f32,
    /// Cross (X) button
    pub cross: bool,
    /// Circle button
    pub circle: bool,
    /// Square button
    pub square: bool,
    /// Triangle button
    pub triangle: bool,
    /// L1 bumper
    pub l1: bool,
    /// R1 bumper
    pub r1: bool,
    /// PS button
    pub ps_button: bool,
}

/// Deadzone for analog sticks (ignore tiny movements)
const STICK_DEADZONE: f32 = 0.05;

static CONNECTED: AtomicBool = AtomicBool::new(false);
static GAMEPAD: Mutex<GamepadState> = Mutex::new(GamepadState {
    left_x: 0.0,
    left_y: 0.0,
    right_x: 0.0,
    right_y: 0.0,
    l2: 0.0,
    r2: 0.0,
    cross: false,
    circle: false,
    square: false,
    triangle: false,
    l1: false,
    r1: false,
    ps_button: false,
});

/// Normalize a 0..255 stick byte to -1.0..1.0 with deadzone.
fn normalize_stick(raw: u8) -> f32 {
    let centered = (raw as f32 - 128.0) / 127.0;
    if centered.abs() < STICK_DEADZONE {
        0.0
    } else {
        centered.clamp(-1.0, 1.0)
    }
}

/// Parse a DualSense BT HID input report.
fn parse_input_report(data: &[u8]) {
    if data.len() < 10 {
        return;
    }

    let left_x = normalize_stick(data[0]);
    // Y axis is inverted on the controller (0=up, 255=down), flip it
    let left_y = -normalize_stick(data[1]);
    let right_x = normalize_stick(data[2]);
    let right_y = -normalize_stick(data[3]);
    let l2 = data[4] as f32 / 255.0;
    let r2 = data[5] as f32 / 255.0;

    let buttons0 = data[7];
    let buttons1 = data[8];
    let buttons2 = data[9];

    let state = GamepadState {
        left_x,
        left_y,
        right_x,
        right_y,
        l2,
        r2,
        square: (buttons0 & 0x10) != 0,
        cross: (buttons0 & 0x20) != 0,
        circle: (buttons0 & 0x40) != 0,
        triangle: (buttons0 & 0x80) != 0,
        l1: (buttons1 & 0x01) != 0,
        r1: (buttons1 & 0x02) != 0,
        ps_button: (buttons2 & 0x01) != 0,
    };

    if let Ok(mut g) = GAMEPAD.lock() {
        *g = state;
    }
}

/// Get the latest gamepad state.
pub fn get_state() -> GamepadState {
    GAMEPAD.lock().map(|g| *g).unwrap_or_default()
}

/// Whether a DualSense controller is currently connected.
pub fn is_connected() -> bool {
    CONNECTED.load(Ordering::Relaxed)
}

/// BT HID Host callback — called by ESP-IDF from the BT task.
unsafe extern "C" fn hidh_callback(event: sys::esp_hidh_cb_event_t, param: *mut sys::esp_hidh_cb_param_t) {
    if param.is_null() {
        return;
    }

    match event {
        sys::esp_hidh_cb_event_t_ESP_HIDH_OPEN_EVT => {
            let open = &(*param).open;
            if open.status == sys::esp_hidh_status_t_ESP_HIDH_OK {
                info!("DualSense connected! handle={}", open.handle);
                CONNECTED.store(true, Ordering::Relaxed);
            } else {
                error!("HID open failed, status={}", open.status);
            }
        }
        sys::esp_hidh_cb_event_t_ESP_HIDH_CLOSE_EVT => {
            info!("DualSense disconnected");
            CONNECTED.store(false, Ordering::Relaxed);
        }
        sys::esp_hidh_cb_event_t_ESP_HIDH_DATA_IND_EVT => {
            let data_evt = &(*param).data_ind;
            if !data_evt.data.is_null() && data_evt.len > 0 {
                let data = std::slice::from_raw_parts(data_evt.data, data_evt.len as usize);
                parse_input_report(data);
            }
        }
        _ => {}
    }
}

/// BT GAP callback for handling pairing/discovery events.
unsafe extern "C" fn gap_callback(event: sys::esp_bt_gap_cb_event_t, param: *mut sys::esp_bt_gap_cb_param_t) {
    if param.is_null() {
        return;
    }

    match event {
        sys::esp_bt_gap_cb_event_t_ESP_BT_GAP_DISC_RES_EVT => {
            let disc = &(*param).disc_res;
            let mut is_dualsense = false;

            for i in 0..disc.num_prop as usize {
                let prop = &*disc.prop.add(i);
                if prop.type_ == sys::esp_bt_gap_dev_prop_type_t_ESP_BT_GAP_DEV_PROP_EIR {
                    let eir = std::slice::from_raw_parts(prop.val as *const u8, prop.len as usize);
                    if let Ok(eir_str) = std::str::from_utf8(eir) {
                        if eir_str.contains("DualSense") || eir_str.contains("Wireless Controller") {
                            is_dualsense = true;
                        }
                    }
                }
                if prop.type_ == sys::esp_bt_gap_dev_prop_type_t_ESP_BT_GAP_DEV_PROP_BDNAME {
                    let name_bytes = std::slice::from_raw_parts(prop.val as *const u8, prop.len as usize);
                    if let Ok(name) = std::str::from_utf8(name_bytes) {
                        let name = name.trim_end_matches('\0');
                        info!("Found BT device: {}", name);
                        if name.contains("DualSense") || name.contains("Wireless Controller") {
                            is_dualsense = true;
                        }
                    }
                }
            }

            if is_dualsense {
                info!("DualSense found! Connecting...");
                // Stop discovery before connecting
                sys::esp_bt_gap_cancel_discovery();
                // Connect via HID Host
                sys::esp_bt_hid_host_connect(disc.bda.as_ptr() as *mut u8);
            }
        }
        sys::esp_bt_gap_cb_event_t_ESP_BT_GAP_DISC_STATE_CHANGED_EVT => {
            let state_evt = &(*param).disc_st_chg;
            if state_evt.state == sys::esp_bt_gap_discovery_state_t_ESP_BT_GAP_DISCOVERY_STOPPED {
                if !is_connected() {
                    info!("Discovery stopped, restarting scan...");
                    sys::esp_bt_gap_start_discovery(
                        sys::esp_bt_inq_mode_t_ESP_BT_INQ_MODE_GENERAL_INQUIRY,
                        10, // 10 * 1.28s = ~13s
                        0,
                    );
                }
            }
        }
        sys::esp_bt_gap_cb_event_t_ESP_BT_GAP_AUTH_CMPL_EVT => {
            let auth = &(*param).auth_cmpl;
            if auth.stat == sys::esp_bt_status_t_ESP_BT_STATUS_SUCCESS {
                info!("BT authentication complete");
            } else {
                warn!("BT authentication failed, status={}", auth.stat);
            }
        }
        sys::esp_bt_gap_cb_event_t_ESP_BT_GAP_PIN_REQ_EVT => {
            // DualSense doesn't normally use PIN, but handle it just in case
            let pin: [u8; 4] = [0, 0, 0, 0];
            sys::esp_bt_gap_pin_reply(
                (*param).pin_req.bda.as_ptr() as *mut u8,
                true,
                4,
                pin.as_ptr() as *mut u8,
            );
        }
        sys::esp_bt_gap_cb_event_t_ESP_BT_GAP_CFM_REQ_EVT => {
            // Auto-accept SSP numeric comparison
            info!("SSP confirm request — auto-accepting");
            sys::esp_bt_gap_ssp_confirm_reply(
                (*param).cfm_req.bda.as_ptr() as *mut u8,
                true,
            );
        }
        sys::esp_bt_gap_cb_event_t_ESP_BT_GAP_KEY_NOTIF_EVT => {
            info!("SSP passkey notification");
        }
        _ => {}
    }
}

/// Initialize Bluetooth and start scanning for a DualSense controller.
pub fn init() -> anyhow::Result<()> {
    unsafe {
        // Release BLE memory since we only use Classic BT
        let ret = sys::esp_bt_controller_mem_release(sys::esp_bt_mode_t_ESP_BT_MODE_BLE);
        if ret != 0 {
            warn!("Failed to release BLE memory: {}", ret);
        }

        // Init and enable BT controller
        let mut bt_cfg = sys::esp_bt_controller_config_t {
            magic: sys::ESP_BT_CONTROLLER_CONFIG_MAGIC_VAL,
            ..Default::default()
        };
        // Use the default config macro values
        bt_cfg.controller_task_stack_size = 4096;
        bt_cfg.controller_task_prio = 23;
        bt_cfg.hci_uart_no = 1;
        bt_cfg.hci_uart_baudrate = 921600;
        bt_cfg.scan_duplicate_mode = 0;
        bt_cfg.scan_duplicate_type = 0;
        bt_cfg.normal_adv_size = 200;
        bt_cfg.mesh_adv_size = 0;
        bt_cfg.send_adv_reserved_size = 1000;
        bt_cfg.controller_debug_flag = 0;
        bt_cfg.mode = sys::esp_bt_mode_t_ESP_BT_MODE_CLASSIC_BT as u8;
        bt_cfg.ble_max_conn = 0;
        bt_cfg.bt_max_acl_conn = 3;
        bt_cfg.bt_max_sync_conn = 3;

        let ret = sys::esp_bt_controller_init(&mut bt_cfg);
        if ret != 0 {
            anyhow::bail!("BT controller init failed: {}", ret);
        }

        let ret = sys::esp_bt_controller_enable(sys::esp_bt_mode_t_ESP_BT_MODE_CLASSIC_BT);
        if ret != 0 {
            anyhow::bail!("BT controller enable failed: {}", ret);
        }

        // Init and enable Bluedroid
        let ret = sys::esp_bluedroid_init();
        if ret != 0 {
            anyhow::bail!("Bluedroid init failed: {}", ret);
        }

        let ret = sys::esp_bluedroid_enable();
        if ret != 0 {
            anyhow::bail!("Bluedroid enable failed: {}", ret);
        }

        // Set device name
        let name = b"DerpyBot\0";
        sys::esp_bt_dev_set_device_name(name.as_ptr());

        // Register GAP callback
        let ret = sys::esp_bt_gap_register_callback(Some(gap_callback));
        if ret != 0 {
            anyhow::bail!("GAP callback register failed: {}", ret);
        }

        // Set SSP IO capability (NoInputNoOutput for auto-pairing)
        let io_cap: u8 = sys::ESP_BT_IO_CAP_NONE as u8;
        sys::esp_bt_gap_set_security_param(
            sys::esp_bt_sp_param_t_ESP_BT_SP_IOCAP_MODE,
            &io_cap as *const u8 as *mut std::ffi::c_void,
            1,
        );

        // Init HID Host
        let ret = sys::esp_bt_hid_host_register_callback(Some(hidh_callback));
        if ret != 0 {
            anyhow::bail!("HID Host callback register failed: {}", ret);
        }

        let ret = sys::esp_bt_hid_host_init();
        if ret != 0 {
            anyhow::bail!("HID Host init failed: {}", ret);
        }

        // Make discoverable and connectable
        sys::esp_bt_gap_set_scan_mode(
            sys::esp_bt_connection_mode_t_ESP_BT_CONNECTABLE,
            sys::esp_bt_discovery_mode_t_ESP_BT_GENERAL_DISCOVERABLE,
        );

        // Start discovery
        info!("Starting Bluetooth discovery — put DualSense in pairing mode (hold Share+PS)...");
        let ret = sys::esp_bt_gap_start_discovery(
            sys::esp_bt_inq_mode_t_ESP_BT_INQ_MODE_GENERAL_INQUIRY,
            10, // 10 * 1.28s = ~13s
            0,
        );
        if ret != 0 {
            anyhow::bail!("BT discovery start failed: {}", ret);
        }

        info!("Bluetooth HID Host initialized, scanning for DualSense...");
    }

    Ok(())
}
