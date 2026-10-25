"""Exercise actual loopback HTTP requests, validation, and durable downloads."""
import http.client
import json
from pathlib import Path
import tempfile
import threading
import unittest

from lumbar.server import Server
from lumbar.service import AppService


class HTTPTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.app = AppService(data_dir=Path(self.temp.name))
        self.server = Server(('127.0.0.1', 0), self.app)
        self.thread = threading.Thread(target=self.server.serve_forever, kwargs={'poll_interval': .01})
        self.thread.start()
        self.addCleanup(self.cleanup)

    def cleanup(self):
        self.server.shutdown()
        self.thread.join(timeout=2)
        self.server.server_close()
        self.app.close()
        self.temp.cleanup()

    def request(self, method, path, payload=None, headers=None):
        connection = http.client.HTTPConnection('127.0.0.1', self.server.server_port, timeout=3)
        body = json.dumps(payload) if payload is not None else None
        request_headers = {'Content-Type': 'application/json'}
        request_headers.update(headers or {})
        connection.request(method, path, body, request_headers)
        response = connection.getresponse()
        result = response.status, dict(response.getheaders()), response.read()
        connection.close()
        return result

    def test_state_has_valid_json_without_external_cors_access(self):
        status, headers, body = self.request('GET', '/api/state')
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(body)['source'], 'simulation')
        self.assertNotIn('Access-Control-Allow-Origin', headers)
        self.assertEqual(headers['Cache-Control'], 'no-store')

    def test_cross_origin_actions_and_untrusted_host_are_rejected(self):
        for headers in ({'Origin': 'https://untrusted.invalid'}, {'Host': 'untrusted.invalid'}, {'Sec-Fetch-Site': 'cross-site'}):
            with self.subTest(headers=headers):
                status, _, _ = self.request('POST', '/api/action', {'action': 'record_start'}, headers)
                self.assertEqual(status, 403)
                self.assertIsNone(self.app.record)

    def test_post_requires_json_and_valid_commands(self):
        self.assertEqual(self.request('POST', '/api/action', {'action': 'stop'}, {'Content-Type': 'text/plain'})[0], 415)
        self.assertEqual(self.request('POST', '/api/action', {'action': 'does_not_exist'})[0], 400)
        self.assertEqual(self.request('POST', '/api/action', {'action': 'parameters', 'values': {'speed': float('nan')}})[0], 400)

    def test_recorded_session_can_be_downloaded_after_stop(self):
        _, _, body = self.request('POST', '/api/action', {'action': 'record_start'})
        sid = json.loads(body)['recording']['id']
        self.app.tick()
        self.request('POST', '/api/action', {'action': 'feedback', 'value': 'comfortable'})
        self.request('POST', '/api/action', {'action': 'record_stop'})
        status, headers, body = self.request('GET', '/api/sessions/' + sid + '/download')
        self.assertEqual(status, 200)
        self.assertIn('attachment', headers['Content-Disposition'])
        session = json.loads(body)
        self.assertEqual(session['id'], sid)
        self.assertGreater(len(session['frames']), 0)
        self.assertTrue(any(e['type'] == 'feedback' for e in session['events']))

    def test_unmapped_paths_cannot_read_workspace_files(self):
        for path in ('/../run.py', '/lumbar/service.py', '/api/sessions/../../run.py'):
            with self.subTest(path=path):
                self.assertIn(self.request('GET', path)[0], (400, 404))


if __name__ == '__main__':
    unittest.main()
