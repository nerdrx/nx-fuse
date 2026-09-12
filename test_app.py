import http.client
import json
import threading
import unittest
from http.server import ThreadingHTTPServer
from app import Simulation, make_handler


class AppTests(unittest.TestCase):
    def test_api_controls_and_local_boundary(self):
        sim=Simulation()
        sim.tick()
        server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(sim))
        thread=threading.Thread(target=server.serve_forever)
        thread.start()
        client=http.client.HTTPConnection('127.0.0.1',server.server_port)
        try:
            for path,body,headers,expected in [
                ('/api/control',{'enabled':True},{},200),
                ('/api/control',{'enabled':'false'},{},400),
                ('/api/control',{'occluded':True},{'Origin':'https://example.com'},403),
                ('/api/control',{'enabled':False},{'Host':'evil.example'},403),
                ('/api/control',{'surprise':True},{},400)]:
                client.request('POST',path,json.dumps(body),headers)
                response=client.getresponse()
                self.assertEqual(response.status,expected)
                response.read()
            client.request('GET','/api/state')
            state=json.loads(client.getresponse().read())
            self.assertTrue(state['enabled'])
            self.assertEqual(state['mode'],'simulation')
            client.request('POST','/api/control',json.dumps({'camera_only':True,'occluded':True}))
            self.assertEqual(client.getresponse().status,200)
            self.assertEqual(set(sim.state['joints']),{'head','left_hand','right_hand'})
            self.assertTrue(sim.state['camera_only'])
        finally:
            client.close()
            server.shutdown()
            server.server_close()
            thread.join()


if __name__ == '__main__':
    unittest.main()
