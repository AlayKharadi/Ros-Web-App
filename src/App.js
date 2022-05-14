import { Suspense, lazy } from 'react';
import { Spinner } from 'react-bootstrap';
import { BrowserRouter, Route, Routes, Navigate } from "react-router-dom";
import CustomNavbar from "./components/CustomNavbar";
const Home = lazy(() => import('./components/Home'));
const Control = lazy(() => import('./components/Control'));
const Logs = lazy(() => import('./components/Logs'))

const App = () => {
    return (
        <BrowserRouter>
            <CustomNavbar></CustomNavbar>
            <Suspense fallback={<Spinner />}>
                <Routes>
                    <Route exact path='/Home' element={<Home />} />
                    <Route exact path='/Control' element={<Control />} />
                    <Route exact path='/Logs' element={<Logs />} />
                    <Route exact path='/' element={<Navigate replace to="/Home" />} />
                </Routes>
            </Suspense>
        </BrowserRouter>
    );
}

export default App;