import io
import unittest
from PIL import Image
from deployment.imagegoal.preview import decode_compressed_rgb


class NativePreviewTests(unittest.TestCase):
    def test_native_jpeg_decodes_without_raw_ros_or_depth(self):
        source=Image.new('RGB',(640,480),(40,110,180))
        buffer=io.BytesIO();source.save(buffer,format='JPEG')
        decoded=decode_compressed_rgb(bytearray(buffer.getvalue()))
        self.assertEqual(decoded.size,(640,480))
        self.assertEqual(decoded.mode,'RGB')
        self.assertLess(sum(abs(a-b) for a,b in zip(decoded.getpixel((100,100)),(40,110,180))),6)

    def test_rejects_empty_or_oversized_frames(self):
        for data in [b'',b'x'*(4*1024*1024+1)]:
            with self.assertRaises(ValueError):decode_compressed_rgb(data)

    def test_rejects_non_jpeg_camera_payload(self):
        buffer=io.BytesIO();Image.new('RGB',(10,10)).save(buffer,format='PNG')
        with self.assertRaises(ValueError):decode_compressed_rgb(buffer.getvalue())


if __name__=='__main__':unittest.main()
