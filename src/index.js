import "bootstrap/dist/css/bootstrap.min.css";
import { StrictMode } from 'react';
import { createRoot } from 'react-dom/client';
import App from './App';
import { Provider } from "react-redux";
import { userStore } from './redux/store';

createRoot(document.getElementById('root'))
  .render(
    <StrictMode>
      <Provider store={userStore}>
        <App />
      </Provider>
    </StrictMode>
  );

// If you want to start measuring performance in your app, pass a function
// to log results (for example: reportWebVitals(console.log))
// or send to an analytics endpoint. Learn more: https://bit.ly/CRA-vitals
