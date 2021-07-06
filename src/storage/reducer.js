import * as action_type from './actiontype';
import ROSLIB from 'roslib';

export default function reducer(state, action) {
    switch (action.type) {
        case action_type.CONNECT: {
            let ros = new ROSLIB.Ros({
                url: action.payload.ws_address
            });
            ros.on('connection', () => {
                state.logs.unshift((new Date()).toTimeString() + ' - Connected!')
                state.connected = true
                state.loading = false
            });
            ros.on('error', (error) => {
                state.logs.unshift((new Date()).toTimeString() + ` - Error: ${error}`)
            });
            ros.on('close', () => {
                state.logs.unshift((new Date()).toTimeString() + ' - Disconnected!')
                state.connected = false
                state.loading = false
            });
            return{
                ...state,
                loading: true,
                ros: ros
            }
        }
        case action_type.DISCONNECT: {
            state.ros.close();
            return {
                ...state,
                ros: null
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
            }       
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