"""Reproduce the browser checks in summary.json using an isolated local lab.

Requires the project environment, Playwright and installed Chromium. For the
optional QA installation used during development, run from the project root:

PYTHONPATH=/tmp/spherical-ui-qa PLAYWRIGHT_BROWSERS_PATH=/tmp/spherical-ui-browsers \
  .venv/bin/python results/yaw_control/browser/reproduce.py

The script starts and stops only its own server, on port 8771 by default.
Injected API responses below test UI rendering only, never physical recovery.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
from urllib.request import urlopen

from playwright.sync_api import expect, sync_playwright


def check(port):
    root = Path(__file__).resolve().parents[3]
    url = f'http://127.0.0.1:{port}'
    with socket.socket() as probe:
        if probe.connect_ex(('127.0.0.1', port)) == 0:
            raise RuntimeError(f'Port {port} is already in use; choose --port with a free port')
    errors, console_errors = [], []
    report = {'browser': 'Chromium, Linux headless; CSP enabled',
              'actual_live_simulation_checks': {}, 'ui_rendering_fixtures_only': {},
              'fixture_scope': 'Injected API responses exercise text/layout only; they neither modify the plant nor demonstrate recovery.'}
    with tempfile.TemporaryFile() as log:
        server = subprocess.Popen([sys.executable, '-m', 'spherical_wheel', 'lab', '--port', str(port)],
                                  cwd=root, stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic()+40
            initial = None
            while time.monotonic() < deadline:
                if server.poll() is not None:
                    log.seek(0)
                    raise RuntimeError(log.read().decode(errors='replace')[-2000:])
                try:
                    with urlopen(url+'/api/state', timeout=1) as response:
                        initial = json.load(response)
                    break
                except OSError:
                    time.sleep(.1)
            if initial is None:
                raise TimeoutError('The isolated lab did not start within 40 seconds')
            with sync_playwright() as playwright:
                browser = playwright.chromium.launch(headless=True, args=['--no-sandbox'])
                context = browser.new_context(viewport={'width': 1440, 'height': 1100})
                page = context.new_page()
                page.on('pageerror', lambda error: errors.append(str(error)))
                page.on('console', lambda message: console_errors.append(message.text) if message.type == 'error' else None)
                response = page.goto(url, wait_until='domcontentloaded')
                assert "script-src 'self'" in response.headers.get('content-security-policy', '')
                expect(page.locator('#connection')).to_contain_text('connected')
                expect(page.locator('#controllerMode')).to_have_value('transition_aware')
                page.locator('#controllerMode').select_option('local_lqr')
                expect(page.locator('#synchronizationToggle')).to_be_disabled()
                expect(page.locator('#synchronizationStatus')).to_contain_text('inactive')
                page.locator('#controllerMode').select_option('transition_aware')
                expect(page.locator('#synchronizationToggle')).to_be_enabled()
                page.locator('label[for="synchronizationToggle"]').click()
                expect(page.locator('#synchronizationToggle')).not_to_be_checked()
                page.locator('label[for="synchronizationToggle"]').click()
                expect(page.locator('#synchronizationToggle')).to_be_checked()
                report['actual_live_simulation_checks']['controller_mode_and_preparation_switches'] = 'passed'
                expect(page.locator('#yawRateValue')).to_have_text('0.0')
                expect(page.locator('#headingDetail')).to_contain_text('Heading 0.0°')
                report['actual_live_simulation_checks']['heading_and_world_spin_present'] = {
                    'heading_deg': initial['heading_deg'], 'yaw_rate_rad_s': initial['yaw_rate_rad_s']}
                page.locator('#playButton').click()
                for _ in range(2):
                    pushed = context.request.post(url+'/api/command', data={
                        'action': 'push', 'force_N': [0., .5, 0.], 'duration_s': .5})
                    assert pushed.ok
                expect(page.locator('#forceBadge')).to_have_text('1.00 N NET · 2 PUSHES')
                measured = context.request.get(url+'/api/state').json()
                assert measured['active_push_count'] == 2
                assert measured['active_force_N'] == [0., 1., 0.]
                report['actual_live_simulation_checks']['overlapping_pushes'] = {
                    'pulse_force_N': [0., .5, 0.], 'pulse_duration_s': .5, 'pulse_count': 2,
                    'measured_active_push_count': measured['active_push_count'],
                    'measured_net_force_N': measured['active_force_N'],
                    'badge': page.locator('#forceBadge').inner_text(),
                    'scope': 'Verifies real force application and display; no recovery claim.'}
                assert context.request.post(url+'/api/command', data={'action': 'pause'}).ok
                exported = context.request.get(url+'/api/export').text()
                assert 'heading_deg,heading_unwrapped_deg,yaw_rate_rad_s' in exported.splitlines()[0]
                report['actual_live_simulation_checks']['csv_heading_spin_columns'] = 'passed'
                page.screenshot(path=str(Path(__file__).parent/'desktop.png'), full_page=True)
                # This response is deliberately synthetic, solely to test UI
                # states that would otherwise require precise sampling timing.
                fixture = dict(initial, running=True, tilt_deg=.01, motion_settled=False,
                               angular_speed_rad_s=7., tracking_velocity_error_m_s=.1,
                               heading_deg=-179.5, heading_unwrapped_deg=540.5, yaw_rate_rad_s=6.283185307179586,
                               active_force_N=[0., 0., 0.], active_push_count=2,
                               synchronization={'targets': [{'name': 'belt_A', 'blend_weight': 1.,
                                                            'actual_mismatch_to_outgoing_m_s': [0., .08, 0.]}]})
                page.route('**/api/state', lambda route: route.fulfill(json=fixture))
                expect(page.locator('#yawRateValue')).to_have_text('360.0')
                expect(page.locator('#headingDetail')).to_have_text('Heading -179.5° · free to turn')
                report['ui_rendering_fixtures_only']['world_spin_converted_to_deg_s'] = True
                expect(page.locator('#tiltStatus')).to_have_text('Recovering')
                expect(page.locator('#runStatus')).to_have_text('Correcting motion')
                expect(page.locator('#forceBadge')).to_have_text('0.00 N NET · 2 PUSHES')
                expect(page.locator('#synchronizationStatus')).to_contain_text('Preparing: Belt A · mismatch 0.080 m/s')
                report['ui_rendering_fixtures_only'].update(
                    upright_but_moving_remains_recovering=True,
                    cancelling_overlaps_show_zero_net_force_and_count=True,
                    incoming_patch_and_actual_mismatch_caption=True)
                page.set_viewport_size({'width': 390, 'height': 844})
                assert page.evaluate('document.documentElement.scrollWidth <= window.innerWidth')
                report['mobile_no_horizontal_overflow'] = True
                page.screenshot(path=str(Path(__file__).parent/'mobile.png'), full_page=True)
                assert not errors, errors
                assert not console_errors, console_errors
                report['online_page_errors'] = list(errors)
                report['online_console_errors'] = list(console_errors)
                page.unroute('**/api/state')
                page.route('**/api/state', lambda route: route.abort())
                expect(page.locator('#controllerMode')).to_be_disabled()
                expect(page.locator('#synchronizationToggle')).to_be_disabled()
                assert not errors, errors
                assert all('net::ERR_FAILED' in error for error in console_errors), console_errors
                report['ui_rendering_fixtures_only']['offline_controls_disabled'] = True
                report['expected_offline_network_error_count'] = len(console_errors)
                browser.close()
            report['status'] = 'passed'
        finally:
            if server.poll() is None:
                server.send_signal(signal.SIGINT)
                try:
                    server.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    server.kill()
                    server.wait(timeout=3)
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8771)
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parent/'summary.json')
    arguments = parser.parse_args()
    result = check(arguments.port)
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
