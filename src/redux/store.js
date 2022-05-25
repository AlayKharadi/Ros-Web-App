import { legacy_createStore } from 'redux';
import reducer from './reducer';

//creating the store using the given reducer function
let initialuserStore = {
    connected: false,
    ros: null,
    ws_address: '',
    logs: [],
    loading: false,
    topic: null,
    message: null,
};

//create the global storage
export const userStore = legacy_createStore(
    reducer,
    initialuserStore,
    window.__REDUX_DEVTOOLS_EXTENSION__ && window.__REDUX_DEVTOOLS_EXTENSION__()
);