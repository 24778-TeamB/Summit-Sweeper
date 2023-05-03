import requests
from typing import Tuple, Dict
from std_msgs.msg import UInt8MultiArray

class horizontalSpeeds(dict):
    def __init__(self, url: str):
        super().__init__()
        frontRight, frontLeft, midRight, midLeft, rearRight, rearLeft = self._load_speed_configs(url)
        _preparedMessages = {
            'forward': UInt8MultiArray(data=[3, frontRight['forward'], frontLeft['forward'], midRight['forward'],
                                             midLeft['forward'], rearRight['forward'], rearLeft['forward']]),
            'reverse': UInt8MultiArray(data=[4, frontRight['reverse'], frontLeft['reverse'], midRight['reverse'],
                                             midLeft['reverse'], rearRight['reverse'], rearLeft['reverse']]),
            'left': UInt8MultiArray(data=[2, frontRight['left'], frontLeft['left'], midRight['left'],
                                          midLeft['left'], rearRight['left'], rearLeft['left']]),
            'right': UInt8MultiArray(data=[1, frontRight['right'], frontLeft['right'], midRight['right'],
                                           midLeft['right'], rearRight['right'], rearLeft['right']]),
            'cw': UInt8MultiArray(data=[3, frontRight['cw'], frontLeft['cw'], midRight['cw'],
                                        midLeft['cw'], rearRight['cw'], rearLeft['cw']]),
            'ccw': UInt8MultiArray(data=[3, frontRight['ccw'], frontLeft['ccw'], midRight['ccw'],
                                         midLeft['ccw'], rearRight['ccw'], rearLeft['ccw']]),
            'climb': UInt8MultiArray(data=[3, frontRight['climb'], frontLeft['climb'], midRight['climb'],
                                           midLeft['climb'], rearRight['climb'], rearLeft['climb']]),
            'stop': UInt8MultiArray(data=[0, frontRight['stop'], frontLeft['stop'], midRight['stop'],
                                          midLeft['stop'], rearRight['stop'], rearLeft['stop']]),
        }
        self.__dict__ = _preparedMessages

    @staticmethod
    def _load_speed_configs(url) -> Tuple[Dict[str, int], Dict[str, int], Dict[str, int], Dict[str, int], Dict[str, int], Dict[str, int]]:
        frontRight = {}
        frontLeft = {}
        midRight = {}
        midLeft = {}
        rearRight = {}
        rearLeft = {}
        f = requests.get(url)
        data = f.json()
        data = data['motor-speeds']
        for motor in data:
            if motor['motor-set'] == 'front' and motor['side'] == 'right':
                for speeds in motor['speeds']:
                    frontRight[speeds['movement']] = speeds['speed']
            if motor['motor-set'] == 'front' and motor['side'] == 'left':
                for speeds in motor['speeds']:
                    frontLeft[speeds['movement']] = speeds['speed']
            if motor['motor-set'] == 'middle' and motor['side'] == 'right':
                for speeds in motor['speeds']:
                    midRight[speeds['movement']] = speeds['speed']
            if motor['motor-set'] == 'middle' and motor['side'] == 'left':
                for speeds in motor['speeds']:
                    midLeft[speeds['movement']] = speeds['speed']
            if motor['motor-set'] == 'rear' and motor['side'] == 'right':
                for speeds in motor['speeds']:
                    rearRight[speeds['movement']] = speeds['speed']
            if motor['motor-set'] == 'rear' and motor['side'] == 'left':
                for speeds in motor['speeds']:
                    rearLeft[speeds['movement']] = speeds['speed']
        return frontRight, frontLeft, midRight, midLeft, rearRight, rearLeft

    def __getitem__(self, key):
        return self.__dict__[key]

    def __repr__(self):
        return repr(self.__dict__)
