import { lazy } from 'react';

const Home = lazy(() => import('../components/Home'));
const Control = lazy(() => import('../components/Control'));
const Logs = lazy(() => import('../components/Logs'));

export const $navlinks = [
    { id: 0, path: 'Home', name: 'Home', element: <Home /> },
    { id: 1, path: 'Control', name: 'Control', element: <Control />  },
    { id: 2, path: 'Logs', name: 'Logs', element: <Logs />  }
];