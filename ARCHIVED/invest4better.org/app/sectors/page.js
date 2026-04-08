"use client"; // Add this at the very top of your file

import React, { useState } from "react";
import { Dialog, DialogPanel } from "@headlessui/react";
import CircularPacking from "../../components/circularpacking";
import Treemap from "../../components/treemap";

import { Bars3Icon, XMarkIcon } from "@heroicons/react/24/outline";
import Image from "next/image";
import Link from "next/link";

const navigation = [{ name: "Home", href: "/" }];

export default function ChatPage() {
  const [mobileMenuOpen, setMobileMenuOpen] = useState(false);

  const [selectedSector, setSelectedSector] = useState(null);

  const stockSectorsData = [
    { sector: "Technology", marketCap: 18.865, esgScore: 13.58 },
    { sector: "Basic Materials", marketCap: 1.6, esgScore: 23.07 },
    { sector: "Consumer Cyclical", marketCap: 6.593, esgScore: 17.2 },
    { sector: "Healthcare", marketCap: 6.561, esgScore: 16.78 },
    { sector: "Energy", marketCap: 3.053, esgScore: 29.4 },
    { sector: "Industrials", marketCap: 5.261, esgScore: 19.84 },
    { sector: "Utilities", marketCap: 1.3, esgScore: 23.37 },
    { sector: "Real Estate", marketCap: 1.477, esgScore: 11.82 },
    { sector: "Consumer Defensive", marketCap: 3.338, esgScore: 23.32 },
    { sector: "Communication Services", marketCap: 5.531, esgScore: 12.35 },
    { sector: "Financial Services", marketCap: 8.587, esgScore: 20.23 },
  ];

  const handleSectorClick = (sectorData) => {
    setSelectedSector(sectorData);
  };

  return (
    <>
      <header className="absolute inset-x-0 top-0 z-10">
        <nav
          aria-label="Global"
          className="mx-auto flex items-center justify-between p-4 sm:p-6 lg:px-8"
        >
          <div className="flex lg:flex-1">
            <Link href="/">
              <Image
                alt="Your Company"
                src="/logo.svg"
                width={200}
                height={200}
                className="h-12 sm:h-16 rounded-full w-auto"
              />
            </Link>
          </div>
          <div className="flex lg:hidden">
            <button
              type="button"
              onClick={() => setMobileMenuOpen(true)}
              className="-m-2.5 inline-flex items-center justify-center rounded-md p-2.5 text-green-500"
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
                className="text-sm font-semibold leading-6 text-green-500 hover:underline rounded-lg px-3 py-2"
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
          <div className="fixed inset-0 z-10" />
          <DialogPanel className="fixed inset-y-0 right-0 z-10 w-full overflow-y-auto bg-gray-900 px-6 py-6 sm:max-w-sm sm:ring-1 sm:ring-gray-900/10">
            <div className="flex items-center justify-between">
              <Link href="/">
                <Image
                  alt="Your Company"
                  src="/logo.svg"
                  width={200}
                  height={200}
                  className="h-12 sm:h-16 rounded-full w-auto"
                />
              </Link>
              <button
                type="button"
                onClick={() => setMobileMenuOpen(false)}
                className="-m-2.5 rounded-md p-2.5 text-gray-700"
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
                      className="-mx-3 block rounded-lg px-3 py-2 text-base font-semibold leading-7 text-green-700 hover:underline"
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

      <div className="bg-gray-900 h-screen w-full overflow-auto relative isolate pt-20 sm:pt-28 lg:px-8">
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

        <div className="lg:flex mx-4 sm:mx-8 lg:mx-0 lg:space-x-12">
          <div className="w-fit">
            <h1 className="text-left mb-4 sm:mb-6 text-green-600 text-3xl sm:text-4xl font-bold">
              Select a Sector to Continue
            </h1>
            <div className="w-full">
              <p className="text-left mb-4 sm:mb-8 text-gray-200">
                <strong>ESG Score</strong>, or{" "}
                <strong>Environmental, Social, and Governance Score</strong>, is
                a rating that evaluates a company's performance on{" "}
                <strong>sustainability</strong> and{" "}
                <strong>ethical practices</strong>. It assesses factors such as{" "}
                <strong>environmental impact</strong>,{" "}
                <strong>social responsibility</strong>, and the effectiveness of{" "}
                <strong>governance structures</strong>. A higher ESG score
                indicates a company's commitment to{" "}
                <strong>responsible business practices</strong>, which can
                attract <strong>investors</strong> and enhance its{" "}
                <strong>reputation</strong>.
              </p>

              <p className="text-left mb-2 sm:mb-4 text-gray-200">
                Larger circles indicate a{" "}
                <strong>bigger market capitalization</strong>. The color coding
                represents the average ESG (Environmental, Social, and
                Governance) scores of all 11 sectors relative to each other.
              </p>
              <ul className="text-left text-gray-400 space-y-2 p-0">
                <li>
                  <span className="text-green-500 font-bold">Light Green:</span>{" "}
                  Highly Above Average Sustainability Practices
                </li>
                <li>
                  <span className="text-green-700 font-bold">Dark Green:</span>{" "}
                  Above Average Sustainability Practices
                </li>
                <li>
                  <span className="text-yellow-500 font-bold">Yellow:</span>{" "}
                  Average Sustainability Practices
                </li>
                <li>
                  <span className="text-orange-500 font-bold">Orange:</span>{" "}
                  Questionable Sustainability Practices
                </li>
                <li>
                  <span className="text-red-600 font-bold">Red:</span>{" "}
                  Problematic Sustainability Practices
                </li>
              </ul>
            </div>

            {/* Display the selected sector info */}
          </div>

          {/* Pass the handleSectorClick to CircularPacking */}
          <div className="">
            <div className="w-full hidden lg:flex  overflow-x-auto lg:justify-center">
              <CircularPacking
                data={stockSectorsData}
                onClickSector={handleSectorClick}
              />
            </div>
            <div className="mt-8 w-full flex lg:hidden  overflow-x-hidden overflow-y-hidden lg:justify-center">
              <Treemap
                data={stockSectorsData}
                onClickSector={handleSectorClick}
              />
            </div>
          </div>
        </div>
      </div>
    </>
  );
}
