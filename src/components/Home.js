import style from '../styles/style.module.css';
import { lazy, Suspense, useState } from "react";
import { Tabs, Tab, Container, Spinner } from "react-bootstrap";
import CoolRobot from '../assets/Cool-Robot.svg';
const Offline = lazy(() => import('./Offline'));
const Online = lazy(() => import('./Online'));


const Home = () => {
	console.log("home")
	const [key, setKey] = useState('offline');

	return (
		<Container className={style.container + " " + style.home}>
			<div>
				<h3>Introduction:</h3>
				<ul>
					<li>We have created an online website through which you can control the ROS robot.</li>
					<li>Below you have instructions for how to use the website both in online or offline mode.</li>
					<li>you can find controller in control section.</li>
					<li>logs regarding the connection process are in logs section.</li>
				</ul>
			</div>
			<img src={CoolRobot} alt="robot-img" />
			<div>
				<Tabs
					activeKey={key}
					defaultActiveKey={key}
					onSelect={(k) => setKey(k)}
					mountOnEnter={true}
					unmountOnExit={true}
					fill
					justify
				>
					<Tab className='mt-3' eventKey="offline" title="Offline">
						<Suspense fallback={<Spinner />}>
							<Offline />
						</Suspense>
					</Tab>
					<Tab className='mt-3' eventKey="online" title="Online">
						<Suspense fallback={<Spinner />}>
							<Online />
						</Suspense>
					</Tab>
				</Tabs>
			</div>
		</Container>
	);
}

export default Home;
