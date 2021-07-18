import * as action_type from './actiontype';
import ROSLIB from 'roslib';

export default function reducer(state, action) {
    switch (action.type) {
        case action_type.CONNECT: {
            return {
                ...state,
                ros: action.payload.ros,
                ws_address: action.payload.ws,
                connected: true,
                loading: false,
                logs: [
                    ...state.logs,
                    {
                        text: ((new Date()).toTimeString() + ' - Connected!'),
                        type: 'connect'
                    }
                ]
            };
        }
            
        case action_type.DISCONNECT: {
            return {
                ...state,
                ros: null,
                ws_address: null,
                connected: false,
                loading: false,
                logs: [
                    ...state.logs,
                    {
                        text: ((new Date()).toTimeString() + ' - Disconnected!'),
                        type: 'disconnect'
                    }
                ]
            };
        }
            
        case action_type.ERROR: {
            return {
                ...state,
                logs: [
                    ...state.logs,
                    {
                        text: ((new Date()).toTimeString() + ` - Error: ${JSON.stringify(action.payload.error)}`),
                        type: 'error'
                    }
                ]
            };
        }
        
        case action_type.SET_TOPIC: {
            return {
                ...state,
                topic: new ROSLIB.Topic({
                    ros: state.ros,
                    name: '/cmd_vel',
                    messageType: 'geometry_msgs/Twist'
                })
            };
        }
        
        case action_type.FORWARD: {
            return {
                ...state,
                message: new ROSLIB.Message({
                    linear: { x: 1, y: 0, z: 0, },
                    angular: { x: 0, y: 0, z: 0, },
                })
            };
        }
        
        case action_type.BACKWARD: {
            return {
                ...state,
                message: new ROSLIB.Message({
                    linear: { x: -1, y: 0, z: 0, },
                    angular: { x: 0, y: 0, z: 0, },
                })
            };
        }
            
        case action_type.TURNLEFT: {
            return {
                ...state,
                message: new ROSLIB.Message({
                    linear: { x: 0.5, y: 0, z: 0, },
                    angular: { x: 0, y: 0, z: 0.5, },
                })
            };
        }
            
        case action_type.TURNRIGHT: {
            return {
                ...state,
                message: new ROSLIB.Message({
                    linear: { x: 0.5, y: 0, z: 0, },
                    angular: { x: 0, y: 0, z: -0.5, },
                })
            };
        }
            
        case action_type.STOP: {
            return {
                ...state,
                message: new ROSLIB.Message({
                    linear: { x: 0, y: 0, z: 0, },
                    angular: { x: 0, y: 0, z: 0, },
                })
            };
        }
            
            
        default: {
            return state;
        }
    }
}