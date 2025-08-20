/* eslint-disable react/no-unescaped-entities */
"use client";

import { useState, useEffect } from "react";

import { Dialog, DialogPanel } from "@headlessui/react";
import { Bars3Icon, XMarkIcon } from "@heroicons/react/24/outline";
import Image from "next/image";
import Footer from "../components/footer";
import Link from "next/link";

const navigation = [
  { name: "Home", href: "/" },
  // { name: "Tools", href: "/tools" },
  // { name: "Bots", href: "/chat" },
  { name: "Find Stocks", href: "/sectors" },
];

export default function Example() {
  const [mobileMenuOpen, setMobileMenuOpen] = useState(false);
  const [isUser, setIsUser] = useState(true);

  return (
    <div className="bg-gray-900 h-screen min-h-max">
      {/* Header */}
      <header className="absolute inset-x-0 top-0 z-50">
        <nav
          aria-label="Global"
          className="mx-auto flex  items-center justify-between p-6 lg:px-8"
        >
          <div className="flex lg:flex-1">
            <Link href="/">
              <Image
                alt="Your Company"
                src="/logo.svg"
                width={200}
                height={200}
                className="h-16 rounded-full w-auto"
              />
            </Link>
          </div>
          <div className="flex lg:hidden">
            <button
              type="button"
              onClick={() => setMobileMenuOpen(true)}
              className="-m-2.5 inline-flex items-center justify-center rounded-md p-2.5 text-green-600"
            >
              <span className="sr-only">Open main menu</span>
              <Bars3Icon aria-hidden="true" className="h-6 w-6" />
            </button>
          </div>
          <div className="hidden lg:flex lg:gap-x-12">
            {navigation.map((item) => (
              <Link
                key={item.name}
                href={item.href}
                className="text-sm font-semibold leading-6 text-green-600 hover:underline  rounded-lg px-3 py-2"
              >
                {item.name}
              </Link>
            ))}
          </div>
        </nav>
        <Dialog
          open={mobileMenuOpen}
          onClose={setMobileMenuOpen}
          className="lg:hidden"
        >
          <div className="fixed inset-0 z-50" />
          <DialogPanel className="fixed inset-y-0 right-0 z-50 w-full overflow-y-auto bg-gray-900 px-6 py-6 sm:max-w-sm sm:ring-1 sm:ring-gray-900/10">
            <div className="flex items-center justify-between">
              <Link href="/">
                <Image
                  alt="Your Company"
                  src="/logo.svg"
                  width={200}
                  height={200}
                  className="h-16 rounded-full w-auto"
                />
              </Link>
              <button
                type="button"
                onClick={() => setMobileMenuOpen(false)}
                className="-m-2.5 rounded-md p-2.5 text-gray-50"
              >
                <span className="sr-only">Close menu</span>
                <XMarkIcon aria-hidden="true" className="h-6 w-6" />
              </button>
            </div>
            <div className="mt-6 flow-root">
              <div className="-my-6 divide-y divide-gray-500/10">
                <div className="space-y-2 py-6">
                  {navigation.map((item) => (
                    <Link
                      key={item.name}
                      href={item.href}
                      className="-mx-3 block rounded-lg px-3 py-2 text-base font-semibold leading-7 text-green-600 hover:underline"
                    >
                      {item.name}
                    </Link>
                  ))}
                </div>
              </div>
            </div>
          </DialogPanel>
        </Dialog>
      </header>

      <div className="relative isolate pt-14 lg:px-8">
        <div
          aria-hidden="true"
          className="absolute inset-x-0 -top-40 -z-10 transform-gpu overflow-hidden blur-3xl sm:-top-80"
        >
          <div
            style={{
              clipPath:
                "polygon(74.1% 44.1%, 100% 61.6%, 97.5% 26.9%, 85.5% 0.1%, 80.7% 2%, 72.5% 32.5%, 60.2% 62.4%, 52.4% 68.1%, 47.5% 58.3%, 45.2% 34.5%, 27.5% 76.7%, 0.1% 64.9%, 17.9% 100%, 27.6% 76.8%, 76.1% 97.7%, 74.1% 44.1%)",
            }}
            className="relative left-[calc(50%-11rem)] aspect-[1155/678] w-[36.125rem] -translate-x-1/2 rotate-[30deg] bg-gradient-to-tr from-emerald-900 to-emerald-700 opacity-30 sm:left-[calc(50%-30rem)] sm:w-[72.1875rem]"
          />
        </div>

        <div className="mx-auto max-w-7xl px-4">
          <div className="grid grid-cols-1 md:grid-cols-2 gap-8 items-center">
            <div className="text-center md:text-left">
              <h1 className="text-4xl font-bold mt-16 tracking-tight text-green-600 sm:text-6xl">
                Invest4Better
              </h1>
              <p className="mt-6 text-lg leading-8 text-gray-50">
                <a
                  className="text-blue-500 hover:cursor-pointer hover:text-blue-600 hover:underline"
                  target="_blank"
                  href="https://www.nrdc.org/bio/josh-axelrod/corporate-honesty-and-climate-change-time-own-and-act"
                >
                  The 15 largest food and beverage companies in the United
                  States are responsible for nearly 630 million metric tons of
                  greenhouse gases emitted each year.{" "}
                </a>
                Environmental Sustainability is one of the{" "}
                <span className="font-bold text-green-600">
                  most pressing conversations
                </span>{" "}
                happening everywhere—in our schools, homes, on the news, and
                across social media. We all want the beautiful trees of the
                Catskills and the natural landscapes we cherish to be around for
                future generations. That’s why we believe the best way to tackle
                this issue is{" "}
                <span className="font-bold text-green-600">at the source:</span>{" "}
                the companies responsible for large-scale emissions.
              </p>
              <p className="mt-6 text-lg leading-8 text-gray-50">
                For example, with approximately{" "}
                <a
                  href="https://www.nasdaq.com/articles/amazon-stocks-nasdaq:amzn-ownership:-institutional-investors-lead-the-pack"
                  target="_blank"
                  className="text-blue-500 hover:cursor-pointer hover:text-blue-600 hover:underline"
                >
                  43% of Amazon’s stock held by individual investors
                </a>{" "}
                like me and you, we can make a real impact by supporting
                companies that prioritize the environment. We've learned in our
                business classes and from our parents that investing in
                companies can benefit our financial future.{" "}
                <span className="text-green-600 underline font-bold ">
                  Our goal is to help people make investments that are not only
                  good for their financial future but also beneficial for the
                  future of the planet.
                </span>{" "}
              </p>
              <div className="mt-10 mb-16 flex items-center justify-center md:justify-start gap-x-6">
                <Link
                  href="/sectors"
                  className="rounded-md bg-green-700 px-3.5 py-2.5 text-sm font-semibold text-white shadow-sm hover:bg-green-600 focus-visible:outline focus-visible:outline-2 focus-visible:outline-offset-2 focus-visible:outline-indigo-600"
                >
                  Get Started Today
                </Link>
                {/* <Link
                  href="/about"
                  className="text-sm font-semibold leading-6 text-green-700"
                >
                  Learn more <span aria-hidden="true">→</span>
                </Link> */}
              </div>
            </div>
            <div className="flex h-fit justify-center md:justify-end">
              <Image
                alt="Your Company"
                src="/image.svg"
                width={500}
                priority={true}
                height={500}
                className="h-auto"
              />
            </div>
          </div>
        </div>
      </div>

      <div
        aria-hidden="true"
        className="absolute inset-x-0 top-[calc(100%-30rem)] -z-10 transform-gpu overflow-hidden blur-3xl sm:top-[calc(100%-45rem)]"
      >
        <div
          style={{
            clipPath:
              "polygon(74.1% 44.1%, 100% 61.6%, 97.5% 26.9%, 85.5% 0.1%, 80.7% 2%, 72.5% 32.5%, 60.2% 62.4%, 52.4% 68.1%, 47.5% 58.3%, 45.2% 34.5%, 27.5% 76.7%, 0.1% 64.9%, 17.9% 100%, 27.6% 76.8%, 76.1% 97.7%, 74.1% 44.1%)",
          }}
          className="relative left-[calc(50%-11rem)] aspect-[1155/678] w-[36.125rem] -translate-x-1/2 rotate-[30deg] bg-gradient-to-tr from-emerald-900 to-emerald-700 opacity-30 sm:left-[calc(50%-30rem)] sm:w-[72.1875rem]"
        />
      </div>

      {/* Footer */}
      {/* <Footer /> */}
    </div>
  );
}
