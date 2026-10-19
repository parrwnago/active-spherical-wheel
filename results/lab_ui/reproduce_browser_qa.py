"""Optional browser QA; requires Playwright + Chromium and lab running on :8765."""
import csv
import json
from pathlib import Path
import time
from urllib.request import urlopen
from playwright.sync_api import sync_playwright,expect

OUT=Path(__file__).resolve().parent
URL='http://127.0.0.1:8765'

def state():
    return json.load(urlopen(URL+'/api/state',timeout=10))

def wait_state(predicate,timeout=25):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        value=state()
        if predicate(value):return value
        time.sleep(.06)
    raise AssertionError('Simulation state condition timed out: '+str({k:value[k] for k in ('time_s','running','failed','last_push','scenario')}))

def main():
    errors=[];console_errors=[];report={}
    with sync_playwright() as playwright:
        browser=playwright.chromium.launch(headless=True,args=['--no-sandbox'])
        context=browser.new_context(viewport={'width':1440,'height':1050},accept_downloads=True)
        # Establish a repeatable starting state, including persisted camera preferences.
        for payload in ({'action':'pause'}, {'action':'controller','enabled':True},
                        {'action':'belt','enabled':True},
                        {'action':'camera','azimuth':135,'elevation':-23,'distance':.9}):
            response=context.request.post(URL+'/api/command',data=payload)
            assert response.ok,response.text()
        page=context.new_page()
        page.on('pageerror',lambda error:errors.append(str(error)))
        page.on('console',lambda message:console_errors.append(message.text) if message.type=='error' else None)
        page.goto(URL,wait_until='domcontentloaded')
        expect(page.locator('#connection')).to_contain_text('connected')
        page.locator('#streamPlaceholder').wait_for(state='hidden')
        page.locator('[data-scenario="shell"]').click()
        wait_state(lambda value:value['scenario']=='shell' and value['time_s']==0)
        expect(page.locator('#playButton')).to_be_enabled()
        expect(page.locator('#timeValue')).to_have_text('0.00')
        expect(page.locator('#cameraLabel')).to_have_text('Perspective')
        page.wait_for_timeout(400)  # Let the separate renderer and chart paint catch the reset.
        page.screenshot(path=str(OUT/'desktop.png'),full_page=True)
        page.locator('#initialRoll').fill('2');page.locator('#initialPitch').fill('0')
        page.locator('[data-scenario="polar"]').click()
        wait_state(lambda value:value['scenario']=='polar' and abs(value['roll_deg']-2)<.01)
        expect(page.locator('#contactValue')).to_contain_text('Belt A')
        page.locator('#cameraButton').click()
        expect(page.locator('#cameraLabel')).to_have_text('Front view')
        page.locator('#playButton').click()
        wait_state(lambda value:value['time_s']>=3 and value['tilt_deg']<.3)
        page.locator('[data-push="right"]').click()
        push=wait_state(lambda value:value['last_push'] is not None)['last_push']
        assert push['force_N']==[0.,.5,0.]
        polar=wait_state(lambda value:value['time_s']>push['time_s']+3 and value['tilt_deg']<.5)
        assert not polar['failed']
        page.locator('#playButton').click()
        paused=wait_state(lambda value:not value['running'])['time_s'];time.sleep(.25)
        assert state()['time_s']==paused
        page.screenshot(path=str(OUT/'polar_disturbance.png'),full_page=True)
        with page.expect_download() as download:
            page.locator('#saveButton').click()
        csv_path=OUT/'browser_export.csv';download.value.save_as(csv_path)
        rows=list(csv.DictReader(csv_path.open()))
        applied=[row for row in rows if abs(float(row['force_y_N']))>.001]
        assert len(applied)==20,len(applied)
        assert 'requested_belt_A_speed_m_s' in rows[0]
        report['polar_push']={'final_tilt_deg':polar['tilt_deg'],'push':push,'applied_intervals':len(applied),'paused_clock_stable':True}
        page.locator('#playButton').click();wait_state(lambda value:value['running'])
        expect(page.locator('#playButton')).to_be_enabled()
        page.keyboard.press('a')
        wait_state(lambda value:value['last_push']['force_N']==[0.,-.5,0.])
        viewport=page.locator('#viewport');viewport.scroll_into_view_if_needed();box=viewport.bounding_box()
        page.mouse.move(box['x']+box['width']/2,box['y']+box['height']/2)
        page.mouse.down();page.mouse.move(box['x']+box['width']/2+60,box['y']+box['height']/2,steps=6);page.mouse.up()
        wait_state(lambda value:abs(value['last_push']['force_N'][1]-.3)<.001)
        report['keyboard_and_drag_push']=True
        page.locator('#playButton').click();wait_state(lambda value:not value['running'])
        page.locator('label[for="beltToggle"]').click();expect(page.locator('#beltToggle')).not_to_be_checked()
        page.locator('#resetButton').click();wait_state(lambda value:value['time_s']==0 and not value['belt_enabled'])
        page.locator('#playButton').click()
        fallen=wait_state(lambda value:value['failed'])
        page.locator('#failureBanner').wait_for(state='visible')
        expect(page.locator('#playButton')).to_be_disabled()
        page.screenshot(path=str(OUT/'belt_disabled_failure.png'),full_page=True)
        report['belt_disabled_failure']={'time_s':fallen['time_s'],'tilt_deg':fallen['tilt_deg'],'visible_failure':True}
        page.locator('label[for="beltToggle"]').click()
        page.locator('#initialRoll').fill('0');page.locator('#initialPitch').fill('0')
        page.locator('[data-scenario="shell"]').click()
        wait_state(lambda value:value['scenario']=='shell' and value['time_s']==0 and not value['failed'])
        page.locator('[data-path="curve"]').click()
        page.locator('#trajectoryButton').click()
        wait_state(lambda value:value['running'] and value['trajectory']['kind']=='curve' and value['time_s']>1.5)
        page.locator('[data-push="forward"]').click()
        tracked=wait_state(lambda value:value['time_s']>=10.2,timeout=20)
        assert not tracked['failed'] and tracked['tracking_error_m']<.01,tracked['tracking_error_m']
        page.locator('#playButton').click();wait_state(lambda value:not value['running'])
        expect(page.locator('#tiltEmpty')).to_be_hidden()
        expect(page.locator('#pathEmpty')).to_be_hidden()
        page.wait_for_timeout(300)
        page.screenshot(path=str(OUT/'tracking.png'),full_page=True)
        report['tracking_with_push']={key:tracked[key] for key in ('time_s','tilt_deg','tracking_error_m','last_push')}
        page.locator('#helpButton').click();expect(page.locator('#helpDialog')).to_be_visible()
        page.locator('#helpDone').click();expect(page.locator('#helpDialog')).not_to_be_visible()
        mobile=context.new_page()
        mobile.set_viewport_size({'width':390,'height':844})
        mobile.on('pageerror',lambda error:errors.append(str(error)))
        mobile.goto(URL,wait_until='domcontentloaded');expect(mobile.locator('#connection')).to_contain_text('connected')
        mobile.locator('#streamPlaceholder').wait_for(state='hidden')
        expect(mobile.locator('#cameraLabel')).to_have_text('Front view')
        expect(mobile.locator('[data-path="curve"]')).to_have_class('path-option selected')
        expect(mobile.locator('#tiltEmpty')).to_be_hidden()
        expect(mobile.locator('#pathEmpty')).to_be_hidden()
        mobile.wait_for_timeout(300)
        mobile.screenshot(path=str(OUT/'mobile.png'),full_page=True)
        from PIL import Image
        report['mobile_screenshot_size']=list(Image.open(OUT/'mobile.png').size)
        assert report['mobile_screenshot_size'][0]==390,report['mobile_screenshot_size']
        mobile.locator('[data-path="sideways"]').click()
        mobile.locator('#pathDistance').fill('33')
        mobile.wait_for_timeout(700)
        expect(mobile.locator('[data-path="sideways"]')).to_have_class('path-option selected')
        expect(mobile.locator('#pathDistance')).to_have_value('33')
        mobile.locator('[data-scenario="transition"]').click()
        expect(mobile.locator('#scenarioTag')).to_have_text('Transition')
        expect(mobile.locator('[data-path="curve"]')).to_have_class('path-option selected')
        expect(mobile.locator('#pathDistance')).to_have_value('60')
        expect(mobile.locator('#pathLateral')).to_have_value('5')
        report['trajectory_selection_sync']={'reload_matches_existing_curve':True,'same_run_preserves_user_edits':True,
                                             'transition_reset_fields_mm':[60,5],'camera_persistence':True}
        report['page_errors']=errors;report['console_errors']=console_errors
        assert not errors,errors
        assert not console_errors,console_errors
        report['status']='passed';report['browser']='Chromium, Linux headless; CSP kept enabled'
        (OUT/'browser_qa.json').write_text(json.dumps(report,indent=2)+'\n')
        browser.close()
    print(json.dumps(report,indent=2))

if __name__=='__main__':main()
