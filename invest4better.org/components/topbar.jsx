"use client";

import { Dialog, DialogPanel } from "@headlessui/react";
import { Bars3Icon, XMarkIcon } from "@heroicons/react/24/outline";
import Image from "next/image";
import { useState } from "react";
import Link from "next/link";
import { useRouter } from "next/navigation";

export default function TopBar({ navigation }) {
  const router = useRouter();

  const [mobileMenuOpen, setMobileMenuOpen] = useState(false);
  return (
    <header className="bg-white lg:shadow-md z-40 ml-36 lg:ml-0 fixed inset-x-0 top-0">
      <nav
        aria-label="Global"
        className="flex items-center justify-between p-4 lg:px-8"
      >
        <div className="flex flex-1">
          <Link href="/" className="lg:block hidden -m-1.5 p-1.5">
            <Image
              alt="Your Company"
              src="/logo.svg"
              width={200}
              height={200}
              className="mx-auto h-12  rounded-full w-auto"
            />
            <span className="sr-only">Your Company</span>
          </Link>
        </div>

        <div className="hidden lg:flex lg:gap-x-12">
          {navigation.map((item) => (
            <Link
              key={item.name}
              href={item.href}
              className="text-sm font-semibold leading-6 text-green-700 hover:underline !important rounded-lg px-3 py-2"
            >
              {item.name}
            </Link>
          ))}
        </div>

        <div className="flex lg:hidden">
          <button
            type="button"
            onClick={() => setMobileMenuOpen(true)}
            className="-m-2.5 inline-flex items-center justify-center rounded-md p-2.5 text-green-700"
          >
            <span className="sr-only">Open main menu</span>
            <Bars3Icon aria-hidden="true" className="h-6 w-6" />
          </button>
        </div>
      </nav>

      <Dialog
        open={mobileMenuOpen}
        onClose={setMobileMenuOpen}
        className="lg:hidden"
      >
        <div className="fixed inset-0 z-50" />
        <DialogPanel className="fixed inset-y-0 right-0 z-50 w-full overflow-y-auto bg-white px-6 py-6 sm:max-w-sm sm:ring-1 sm:ring-gray-900/10">
          <div className="flex items-center justify-between">
            <Link href="/" className="-m-1.5 p-1.5">
              <span className="sr-only">Your Company</span>
              <Image
                alt="Your Company"
                src="/logo.svg"
                width={200}
                height={200}
                className="mx-auto h-14 rounded-full w-auto"
              />
            </Link>
            <button
              type="button"
              onClick={() => setMobileMenuOpen(false)}
              className="-m-2.5 rounded-md p-2.5 text-green-700"
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
                    className="-mx-3 block rounded-lg px-3 py-2 text-base font-semibold leading-7 text-green-700 hover:underline !important"
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
  );
}
