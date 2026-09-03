import Image from "next/image";
import type { IconType } from "react-icons";
import { FaJava } from "react-icons/fa";
import { LuArrowRight } from "react-icons/lu";
import {
  SiCplusplus,
  SiDocker,
  SiFastapi,
  SiGit,
  SiKotlin,
  SiLinux,
  SiPython,
  SiRaspberrypi,
  SiRos,
  SiRuby,
  SiRubyonrails,
  SiTypescript,
} from "react-icons/si";

const Arrow = () => <LuArrowRight aria-hidden="true" className="ml-2 shrink-0 text-base transition-transform group-hover:translate-x-1" />;

export default function Home() {
  return (
    <main className="overflow-hidden bg-ink text-slate-100">
      <section className="relative min-h-[760px] border-b border-white/10">
        <Image src="/images/joeymarra-city.png" alt="Futuristic city at sunset" fill priority className="object-cover object-center" />
        <div className="absolute inset-0 bg-[linear-gradient(90deg,rgba(3,14,36,.96)_0%,rgba(3,14,36,.78)_38%,rgba(3,14,36,.20)_78%,rgba(3,14,36,.40)_100%)]" />
        <div className="absolute inset-0 bg-[linear-gradient(0deg,rgba(3,14,36,.98)_0%,transparent_44%)]" />
        <div className="relative mx-auto flex min-h-[760px] max-w-7xl flex-col px-6 sm:px-10 lg:px-12">
          <nav className="flex items-center justify-between py-7 text-xs font-semibold uppercase tracking-[0.18em]">
            <div className="flex items-center gap-4">
              <a href="#top" aria-label="Back to top" className="flex items-center gap-3 rounded-lg border border-white/15 bg-white/95 px-3 py-2 shadow-lg shadow-ink/20">
                <Image src="/images/georgia-tech.png" width={82} height={30} alt="Georgia Tech" className="h-7 w-auto object-contain" />
                <div className="hidden border-l border-ink/15 pl-3 text-[8px] font-bold uppercase leading-4 tracking-[.12em] text-ink sm:block">Honors student<br /><span className="text-[#8f6f29]">B.S. CS · 2028</span></div>
              </a>
            </div>
            <div className="hidden gap-7 text-slate-300 md:flex"><a href="#work" className="transition hover:text-cyan">Work</a><a href="#experience" className="transition hover:text-cyan">Experience</a><a href="#skills" className="transition hover:text-cyan">Skills</a><a href="https://github.com/Hiptostee/" target="_blank" rel="noreferrer" className="transition hover:text-cyan">GitHub</a><a href="#contact" className="transition hover:text-cyan">Contact</a></div>
            <a href="mailto:jmarra6@gatech.edu" className="rounded-full border border-white/25 px-4 py-2 transition hover:border-cyan hover:bg-cyan/10">Get in touch</a>
          </nav>
          <div id="top" className="my-auto max-w-3xl pb-24 pt-16 sm:pb-32">
            <p className="mb-6 animate-rise text-xs font-semibold uppercase tracking-[0.22em] text-cyan [animation-delay:100ms]">Robotics · autonomy · systems</p>
            <h1 className="animate-rise text-5xl font-semibold leading-[.95] tracking-[-0.055em] text-white sm:text-7xl lg:text-8xl [animation-delay:180ms]">Software for <span className="text-amber">autonomous systems.</span></h1>
            <p className="mt-7 max-w-xl animate-rise text-base leading-7 text-slate-200 sm:text-lg [animation-delay:280ms]">I&apos;m Joseph Marra, a Georgia Tech CS student building robotics software, embedded control systems, and reliable interfaces between them.</p>
            <div className="mt-9 flex animate-rise flex-wrap gap-3 [animation-delay:360ms]">
              <a href="#work" className="group inline-flex items-center rounded-full bg-amber px-5 py-3 text-sm font-bold text-ink transition hover:bg-[#ffd080]">Explore selected work<Arrow /></a>
              <a href="https://www.linkedin.com/in/joseph-marra-245185273/" target="_blank" rel="noreferrer" className="group inline-flex items-center rounded-full border border-white/30 px-5 py-3 text-sm font-semibold text-white transition hover:border-cyan hover:text-cyan">LinkedIn<Arrow /></a>
              <a href="https://github.com/Hiptostee/" target="_blank" rel="noreferrer" className="group inline-flex items-center rounded-full border border-white/30 px-5 py-3 text-sm font-semibold text-white transition hover:border-cyan hover:text-cyan">GitHub<Arrow /></a>
            </div>
          </div>
          <div className="absolute bottom-8 right-6 hidden text-right text-[10px] font-semibold uppercase tracking-[.2em] text-white/60 sm:block lg:right-12">Atlanta, GA<br /><span className="text-cyan">Georgia Tech CS 🐝 · 2028</span></div>
        </div>
      </section>

      <section id="work" className="relative mx-auto max-w-7xl px-6 py-24 sm:px-10 lg:px-12 lg:py-32">
        <div className="mb-14 flex flex-col justify-between gap-5 sm:flex-row sm:items-end"><div><p className="eyebrow">Selected work</p><h2 className="section-title">Systems worth<br />getting right.</h2></div><p className="max-w-sm text-sm leading-6 text-slate-400">Projects across autonomy, embedded control, computer vision, and collaborative product building.</p></div>

        <article className="group relative grid overflow-hidden rounded-2xl border border-white/10 bg-navy/70 lg:grid-cols-[.9fr_1.1fr]">
          <div className="relative min-h-[340px] overflow-hidden bg-[#173059]"><Image src="/images/paesano-bot.png" alt="Paesano holonomic robot" fill className="object-contain p-8 transition duration-700 group-hover:scale-105" /><div className="absolute inset-0 bg-gradient-to-t from-ink/75 via-transparent to-transparent" /><div className="absolute left-6 top-6 rounded-full border border-cyan/40 bg-ink/70 px-3 py-1 text-[10px] font-bold uppercase tracking-[.16em] text-cyan">Flagship project</div></div>
          <div className="relative p-7 sm:p-10 lg:p-12"><p className="eyebrow text-amber">01 — Autonomous robotics platform</p><h3 className="mt-4 text-4xl font-semibold tracking-[-.045em] text-white sm:text-5xl">Paesano</h3><p className="mt-5 max-w-xl text-base leading-7 text-slate-300">A holonomic indoor robot with a ROS 2/C++ autonomy stack. A Raspberry Pi 5 handles planning and estimation; a Pico runs a 50 Hz cascaded motor controller. The goal was a robot I could measure, debug, and improve on real hardware.</p><div className="mt-8 grid gap-5 sm:grid-cols-2"><Fact title="Perception & pose" text="LD19 LiDAR, BNO085 IMU, wheel encoders, EKF fusion, and a particle-filter localizer written from scratch." /><Fact title="Motion & planning" text="Mecanum drive control, spline trajectories, LQR path tracking, A* planning, and frontier exploration." /><Fact title="Tested on hardware" text="Completed 9/9 navigation routes; rosbag analysis measured 0.9 cm mean cross-track and 0.6 cm final-position error." /><Fact title="Built end to end" text="Custom C++ ROS 2 nodes, embedded firmware, simulation, and a SwiftUI app with a FastAPI/WebSocket bridge." /></div><div className="mt-9 flex flex-wrap gap-x-6 gap-y-3"><a href="https://paesano.joeymarra.com" target="_blank" rel="noreferrer" className="group inline-flex items-center text-sm font-bold text-cyan">Technical writeup<Arrow /></a><a href="https://github.com/Hiptostee/Portfolio/tree/main/Paesano" target="_blank" rel="noreferrer" className="group inline-flex items-center text-sm font-bold text-cyan">Source code<Arrow /></a></div></div>
          <div className="border-t border-white/10 bg-ink/40 p-3 sm:p-5 lg:col-span-2">
            <div className="aspect-video overflow-hidden rounded-xl border border-white/10 bg-black shadow-2xl shadow-black/30"><iframe src="https://www.youtube-nocookie.com/embed/7FdfCe_9ezc" title="Paesano autonomous navigation demo" className="h-full w-full" loading="lazy" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerPolicy="strict-origin-when-cross-origin" allowFullScreen /></div>
            <div className="mt-4 flex items-center justify-between px-1"><p className="eyebrow text-amber">Hardware demo</p><a href="https://www.youtube.com/watch?v=7FdfCe_9ezc" target="_blank" rel="noreferrer" className="group inline-flex items-center text-xs font-bold text-cyan">View on YouTube<Arrow /></a></div>
          </div>
        </article>

        <div className="mt-5 grid gap-5 lg:grid-cols-3">
          <ProjectCard image="/images/vision-tracking.png" eyebrow="Computer vision" title="Vision-based target tracking" text="A camera-to-actuator loop: OpenCV detects a target and streams position updates to an Arduino pan-tilt controller for smooth, real-time centering." />
          <ProjectCard image="/images/swishhh.png" eyebrow="AI ATL 2025" title="SWISHHH!" text="A live NBA prediction game combining real-time game data, webcam gestures, and AI-driven interaction." award="Winner · PrizePicks Track + Matt Steele Track" link="https://devpost.com/software/swishhh" />
          <ProjectCard image="/images/cosmic-collector.png" eyebrow="Robotech hackathon" title="Cosmic Collector" text="A Mars-inspired autonomous cave-navigation robot built under pressure with mecanum drive, vision, LiDAR, IMU, ROS navigation, and ESP32 firmware." link="https://devpost.com/software/cosmic-collector" />
        </div>
      </section>

      <section id="experience" className="border-y border-white/10 bg-[#091b37] px-6 py-24 sm:px-10 lg:px-12 lg:py-32"><div className="mx-auto grid max-w-7xl gap-14 lg:grid-cols-[.7fr_1.3fr]"><div><p className="eyebrow">Background</p><h2 className="section-title">Experience</h2></div><div className="max-w-3xl"><p className="text-2xl font-medium leading-[1.35] tracking-[-.03em] text-slate-100 sm:text-3xl">Honors student at Georgia Tech studying computer science, with concentrations in Intelligence and Devices.</p><div className="mt-10 grid gap-8 border-t border-white/10 pt-8 sm:grid-cols-2"><div><div className="flex items-center gap-4"><Image src="/images/robojackets.png" width={52} height={52} alt="RoboJackets" className="h-12 w-12 object-contain" /><div><h3 className="text-lg font-bold text-white">RoboJackets · RoboNav</h3><p className="text-xs text-cyan">Software Lead · URC</p></div></div><p className="mt-4 text-sm leading-6 text-slate-400">Leading 10–15 student engineers rebuilding a legacy ROS 2 rover autonomy stack for the University Rover Challenge. I coordinate localization, perception, navigation, controls, simulation, integration, and team milestones.</p><a href="https://github.com/RoboJackets/urc-software" target="_blank" rel="noreferrer" className="group mt-5 inline-flex items-center text-sm font-bold text-cyan">View RoboNav repository<Arrow /></a></div><div><div className="flex items-center gap-4"><Image src="/images/prizepicks.jpg" width={52} height={52} alt="PrizePicks" className="h-12 w-12 rounded-xl object-cover" /><div><h3 className="text-lg font-bold text-white">PrizePicks</h3><p className="text-xs text-cyan">Software Engineering Intern · Summer 2026</p></div></div><p className="mt-4 text-sm leading-6 text-slate-400">Built the PrizePicks Wins Feed and led delivery across backend, mobile, product, and analytics. The feature increased profile follows by 116%, profile-button presses by 99%, and profile views by 16%.</p></div></div></div></div></section>

      <section id="skills" className="mx-auto max-w-7xl px-6 py-24 sm:px-10 lg:px-12 lg:py-28"><div className="grid gap-12 lg:grid-cols-[.7fr_1.3fr]"><div><p className="eyebrow">Skills</p><h2 className="section-title">Technical toolkit</h2></div><div><div className="grid grid-cols-3 gap-2 sm:grid-cols-4 lg:grid-cols-6"><TechLogo icon={SiCplusplus} name="C++" color="text-[#76a9dc]" /><TechLogo icon={SiPython} name="Python" color="text-[#ffd343]" /><TechLogo icon={FaJava} name="Java" color="text-[#f89820]" /><TechLogo icon={SiTypescript} name="TypeScript" color="text-[#65a9e9]" /><TechLogo icon={SiRos} name="ROS 2" color="text-cyan" /><TechLogo icon={SiRaspberrypi} name="Raspberry Pi" color="text-[#e15b7a]" /><TechLogo icon={SiDocker} name="Docker" color="text-[#4bb5f5]" /><TechLogo icon={SiLinux} name="Linux" color="text-white" /><TechLogo icon={SiKotlin} name="Kotlin" color="text-violet" /><TechLogo icon={SiRuby} name="Ruby" color="text-[#e35b5b]" /><TechLogo icon={SiRubyonrails} name="Rails" color="text-[#e35b5b]" /><TechLogo icon={SiFastapi} name="FastAPI" color="text-[#57c5a8]" /></div><div className="mt-6 grid gap-px overflow-hidden rounded-2xl border border-white/10 bg-white/10 sm:grid-cols-2"><SkillGroup title="Robotics & autonomy" items="Gazebo, localization, EKF, LiDAR, IMU, odometry, path planning, controls" /><SkillGroup title="Systems & software" items="Microcontrollers, I²C, WebSockets, Git, GitHub, unit testing, technical design" /></div></div></div></section>

      <section id="contact" className="relative px-6 py-24 sm:px-10 lg:px-12"><div className="absolute inset-x-0 top-0 h-px bg-gradient-to-r from-transparent via-cyan to-transparent" /><div className="mx-auto flex max-w-7xl flex-col justify-between gap-10 sm:flex-row sm:items-end"><div><p className="eyebrow">Contact</p><h2 className="mt-4 max-w-3xl text-4xl font-semibold tracking-[-.05em] text-white sm:text-6xl">Get in touch.</h2></div><a href="mailto:jmarra6@gatech.edu" className="group inline-flex shrink-0 items-center rounded-full bg-cyan px-6 py-3 text-sm font-bold text-ink transition hover:bg-white">jmarra6@gatech.edu<Arrow /></a></div><footer className="mx-auto mt-24 flex max-w-7xl flex-col gap-3 border-t border-white/10 pt-6 text-[11px] font-semibold uppercase tracking-[.15em] text-slate-500 sm:flex-row sm:justify-between"><span>Joseph Marra © 2026</span><span>Robotics · autonomy · systems</span></footer></section>
    </main>
  );
}

function Fact({ title, text }: { title: string; text: string }) { return <div className="border-l border-white/15 pl-4"><h4 className="text-sm font-bold text-white">{title}</h4><p className="mt-2 text-sm leading-6 text-slate-400">{text}</p></div>; }

function SkillGroup({ title, items }: { title: string; items: string }) { return <div className="bg-ink p-6 sm:p-7"><h3 className="text-sm font-bold text-white">{title}</h3><p className="mt-3 text-sm leading-6 text-slate-400">{items}</p></div>; }

function TechLogo({ icon: Icon, name, color }: { icon: IconType; name: string; color: string }) { return <div className="group flex min-h-28 flex-col items-center justify-center gap-3 rounded-xl border border-white/10 bg-white/[.025] px-3 py-5 transition duration-300 hover:-translate-y-1 hover:border-cyan/40 hover:bg-white/[.055]"><Icon aria-hidden="true" className={`text-3xl transition-transform duration-300 group-hover:scale-110 ${color}`} /><span className="text-center text-[11px] font-semibold text-slate-400 group-hover:text-white">{name}</span></div>; }

function ProjectCard({ image, eyebrow, title, text, link, award }: { image: string; eyebrow: string; title: string; text: string; link?: string; award?: string }) { const content = <><div className="relative h-52 overflow-hidden"><Image src={image} alt="" fill className="object-cover transition duration-700 group-hover:scale-105" /><div className="absolute inset-0 bg-gradient-to-t from-ink/60 to-transparent" /></div><div className="p-6"><p className="eyebrow text-amber">{eyebrow}</p><h3 className="mt-3 text-2xl font-bold tracking-[-.035em] text-white">{title}</h3><p className="mt-3 text-sm leading-6 text-slate-400">{text}</p>{award && <p className="mt-4 rounded-lg border border-amber/20 bg-amber/[.07] px-3 py-2 text-xs font-bold text-amber">{award}</p>}{link && <span className="mt-5 inline-flex items-center text-sm font-bold text-cyan">Read project<Arrow /></span>}</div></>; return link ? <a href={link} target="_blank" rel="noreferrer" className="group overflow-hidden rounded-2xl border border-white/10 bg-white/[.025] transition hover:-translate-y-1 hover:border-cyan/50">{content}</a> : <article className="group overflow-hidden rounded-2xl border border-white/10 bg-white/[.025] transition hover:-translate-y-1 hover:border-cyan/50">{content}</article>; }
