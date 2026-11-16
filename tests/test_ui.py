import shutil
import subprocess
import unittest
from pathlib import Path


class UiFunctionTests(unittest.TestCase):
    def test_display_models_and_shared_layout(self):
        root = Path(__file__).resolve().parents[1]/'imu_desktop/ui'
        lower = (root/'index.html').read_text(encoding='utf-8')
        self.assertNotIn('data-model-id="leg"', lower)
        self.assertEqual(lower.count('data-model-id='), 2)
        self.assertIn('data-model-id="lower" role="radio" aria-checked="true"', lower)
        self.assertIn("loadModel('lower')", (root/'app.js').read_text(encoding='utf-8'))
        upper = (root/'motion.html').read_text(encoding='utf-8')
        for panel in ('workspace', 'controls-panel', 'viewer-panel', 'metrics-column', 'chart-panel'):
            self.assertIn(panel, lower)
            self.assertIn(panel, upper)
        self.assertIn('src="motion-view.js"', upper)
        for page in ('index.html','motion.html','bench.html'):
            self.assertNotIn('Đo định lượng yêu cầu firmware chất lượng mới', (root/page).read_text(encoding='utf-8'))

    def test_one_viewer_links_every_mode(self):
        root = Path(__file__).resolve().parents[1]/"imu_desktop/ui"
        from html.parser import HTMLParser
        class Links(HTMLParser):
            def __init__(self):
                super().__init__()
                self.targets = set()
            def handle_starttag(self, tag, attributes):
                if tag == 'a':
                    self.targets.add(dict(attributes).get('href'))
        pages = {'index.html', 'bench.html', 'motion.html'}
        for name in pages:
            parsed = Links()
            parsed.feed((root/name).read_text(encoding='utf-8'))
            self.assertTrue(pages-{name} <= parsed.targets, name)

    @unittest.skipUnless(shutil.which('node'), 'Node is needed for navigation guards')
    def test_every_mode_blocks_navigation_while_connected(self):
        root = Path(__file__).resolve().parents[1]/'imu_desktop/ui'
        cases = [('app.js', "for (const id of ['benchLink', 'motionLink'])", 'ui.stop.addEventListener'),
                 ('bench.js', "for(const id of ['gaitLink','motionLink'])", 'new ResizeObserver'),
                 ('motion.js', "document.querySelectorAll('header a')", 'function call(')]
        for name, start, end in cases:
            source = (root/name).read_text(encoding='utf-8')
            guard = source[source.index(start):source.index(end, source.index(start))]
            check = r'''
const assert=require('node:assert/strict'),vm=require('node:vm');
const handlers=[],link={addEventListener:(name,handler)=>{assert.equal(name,'click');handlers.push(handler);}};
let notices=0;
const context={isConnected:false,connected:false,state:{connected:false},
  $:()=>link,document:{querySelectorAll:()=>[link,link]},
  showToast:()=>notices++,toast:()=>notices++,message:()=>notices++};
vm.runInNewContext(GUARD,context);
assert.equal(handlers.length,2);
for(const connected of [false,true]) {
  context.isConnected=context.connected=context.state.connected=connected;
  for(const handler of handlers) {
    let prevented=false;handler({preventDefault:()=>prevented=true});
    assert.equal(prevented,connected);
  }
}
assert.equal(notices,2);
'''
            import json
            result = subprocess.run([shutil.which('node')], input='const GUARD='+json.dumps(guard)+';\n'+check,
                                    text=True, capture_output=True, timeout=10)
            self.assertEqual(result.returncode, 0, name+result.stderr)

    def test_operator_pages_use_vietnamese_without_translating_protocol_ids(self):
        root = Path(__file__).resolve().parents[1]/"imu_desktop/ui"
        from html.parser import HTMLParser
        class Text(HTMLParser):
            def __init__(self):
                super().__init__()
                self.parts = []
            def handle_data(self, data):
                self.parts.append(data)
        for name in ("index.html", "bench.html"):
            source = (root/name).read_text(encoding="utf-8")
            self.assertIn('lang="vi"', source)
            parsed = Text()
            parsed.feed(source)
            visible = " ".join(parsed.parts)
            for untranslated in ("Start measurement", "Start trial", "Motor control", "Gait viewer", "Disconnected", "Current session"):
                self.assertNotIn(untranslated, visible)
        self.assertIn('value="binary"', (root/"bench.html").read_text(encoding="utf-8"))
        self.assertIn('value="knee"', (root/"index.html").read_text(encoding="utf-8"))

    @unittest.skipUnless(shutil.which("node"), "Node is needed for the pure-JS chart check")
    def test_chart_downsampling_preserves_joint_extrema_and_gap_boundaries(self):
        source = (Path(__file__).resolve().parents[1]/"imu_desktop"/"ui"/"app.js").read_text(encoding="utf-8")
        function = source[source.index("function envelopeSamples("):source.index("function setBusy(")]
        check = r'''
const assert = require('node:assert/strict');
const input = Array.from({length:20000},(_,i)=>({t:i*10,hip:0,knee:0,ankle:0,gapBefore:i===11111}));
input[1234].hip=130; input[4321].knee=-20; input[7890].ankle=50;
const output=envelopeSamples(input,300);
assert(output.length < 3000);
assert(output.some(x=>x.hip===130)); assert(output.some(x=>x.knee===-20)); assert(output.some(x=>x.ankle===50));
assert(output.some(x=>x.t===111110 && x.gapBefore)); assert(output.some(x=>x.t===111100));
for(let i=1;i<output.length;i++) assert(output[i].t>output[i-1].t);
'''
        result = subprocess.run([shutil.which("node")], input=function+check, text=True, capture_output=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
