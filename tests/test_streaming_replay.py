"""Real CPU codecs retain selected pixels and exact packet time through stream-copy."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'rule_based_workbench')]
from rgbd_segments import SegmentWriter
from segmented_playback import _browser_parts,_remux_browser,_probe_video


@unittest.skipUnless(shutil.which('ffmpeg')and shutil.which('ffprobe'),'qualified CPU codecs required')
class StreamingReplayTests(unittest.TestCase):
    def test_live_codec_resolves_before_child_environment_is_cleaned(self):
        with tempfile.TemporaryDirectory() as temporary:
            with patch('launch_environment.clean_environment', return_value=dict(os.environ, PATH='/nonexistent')):
                writer = SegmentWriter(temporary, 'head', pixel_format='rgb8',
                                       replay_select=lambda rows: list(range(len(rows))))
                try:
                    writer.add_pixels({'frame': 0}, bytes([40, 90, 150] * 64), None, 8, 8)
                finally:
                    writer.finish()
            self.assertEqual(writer.replay_status()['failed_segments'], 0, writer.replay_status())
            self.assertTrue((Path(temporary) / 'head/000000.rgb.browser.json').exists())

    def test_live_encoding_reuses_packets_and_repairs_changed_selection_or_bytes(self):
        with tempfile.TemporaryDirectory()as temporary:
            root=Path(temporary)
            writer=SegmentWriter(root,'head',pixel_format='rgb8',segment_frames=4,replay_select=lambda rows:[1,2,3])
            try:
                for i in range(4):writer.add_pixels({'frame':i},bytes([40+i*30,90,150]*64),None,8,8)
                manifests=writer.wait_committed()
                self.assertEqual(len(manifests),1)
                sidecar=root/'head/000000.rgb.browser.json'
                self.assertTrue(sidecar.exists(),'encoding completed before recorder.finish')
                warmed=json.loads(sidecar.read_text());self.assertEqual(warmed['indices'],[1,2,3])
                stage=root/'join';stage.mkdir()
                parts,counts=_browser_parts(root,manifests,[[1,2,3]],stage,subprocess.run,'ffmpeg')
                self.assertEqual(counts,dict(reused_segments=1,rebuilt_segments=0))
                output=root/'replay.mp4';offsets=[0,37000,82000]
                _remux_browser(parts,output,offsets,28000,stage,subprocess.run,'ffmpeg')
                _probe_video(output,subprocess.run,offsets,110000)
                packets = _probe_video(output, subprocess.run, offsets, 110000, decode=False)
                self.assertEqual(int(packets['nb_read_packets']), len(offsets))
                with self.assertRaisesRegex(ValueError, 'PTS'):
                    _probe_video(output, subprocess.run, [0, 37002, 82000], 110000, decode=False)
                def pixels(path):
                    return subprocess.check_output(['ffmpeg','-nostdin','-v','error','-i',str(path),
                        '-vsync','0','-f','rawvideo','-pix_fmt','rgb24','pipe:1'])
                self.assertEqual(pixels(output),pixels(parts[0]))
                rebuilt=root/'repair';rebuilt.mkdir()
                parts,counts=_browser_parts(root,manifests,[[2,3]],rebuilt,subprocess.run,'ffmpeg')
                self.assertEqual(counts,dict(reused_segments=0,rebuilt_segments=1))
                _remux_browser(parts,root/'short.mp4',[0,45000],28000,rebuilt,subprocess.run,'ffmpeg')
                _probe_video(root/'short.mp4',subprocess.run,[0,45000],73000)
                damaged=root/warmed['file']['path'];damaged.write_bytes(b'corrupt derivative')
                recovery=root/'recovery';recovery.mkdir()
                _,counts=_browser_parts(root,manifests,[[1,2,3]],recovery,subprocess.run,'ffmpeg')
                self.assertEqual(counts['rebuilt_segments'],1)
            finally:writer.finish()


if __name__=='__main__':unittest.main()
