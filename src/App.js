import { Suspense } from 'react';
import { Spinner } from 'react-bootstrap';
import { BrowserRouter, Route, Routes, Navigate } from "react-router-dom";
import CustomNavbar from "./components/CustomNavbar";
import { $navlinks } from './storage/navlinks';

const App = () => {
    return (
        <BrowserRouter>
            <CustomNavbar />
            <Routes>
                {
                    $navlinks.map((navlink, index) => {
                        return (
                            <Route
                                key={navlink.id}
                                exact={true}
                                path={navlink.path}
                                element={
                                    <Suspense fallback={<Spinner />}>
                                        {navlink.element}
                                    </Suspense>
                                }
                            />
                        )
                    })
                }
                <Route exact path='/' element={<Navigate replace={true} to={`/${$navlinks[0].path}`} />} />
                <Route path='*' element={<Navigate replace={true} to={'/'} />} />
            </Routes>
        </BrowserRouter >
    );
}

export default App;