"use client";

import { useState } from "react";
import Image from "next/image";
import {
  Dialog,
  DialogBackdrop,
  DialogPanel,
  Menu,
  MenuItem,
  MenuItems,
  TransitionChild,
} from "@headlessui/react";
import {
  Bars3Icon,
  XMarkIcon,
  EllipsisVerticalIcon,
} from "@heroicons/react/24/outline";
import Link from "next/link";

import { useRouter } from "next/navigation";

function classNames(...classes) {
  return classes.filter(Boolean).join(" ");
}

export default function SideBar({ navigation }) {
  const [sidebarOpen, setSidebarOpen] = useState(false);
  const [openDropdown, setOpenDropdown] = useState(null);
  const router = useRouter();

  const handleEllipsisClick = (id) => {
    setOpenDropdown(openDropdown === id ? null : id); // Toggle dropdown
  };

  async function deleteChat(id) {
    console.log(id);

    const response = await fetch("/api/deleteChat", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify({
        chatId: id,
      }),
    });

    const data = await response.json();

    if (response.ok) {
      // Then, reload the page after navigation
      window.location.reload();
    } else {
      // Handle errors if needed
      console.error(data.message);
    }
  }

  return (
    <>
      <div className="z-40 w-48 fixed">
        <Dialog
          open={sidebarOpen}
          onClose={() => setSidebarOpen(false)}
          className="relative lg:hidden"
        >
          <DialogBackdrop
            transition
            className="fixed inset-0 bg-white opacity-80 transition-opacity duration-300 ease-linear data-[closed]:opacity-100"
          />

          <div className="fixed inset-0 flex opacity-100">
            <DialogPanel
              transition
              className="relative mr-16 flex w-full max-w-xs flex-1 transform transition duration-300 ease-in-out data-[closed]:-translate-x-full bg-white opacity-100"
            >
              <TransitionChild>
                <div className="absolute left-full top-0 flex w-16 justify-center pt-5 duration-300 ease-in-out data-[closed]:opacity-0">
                  <button
                    type="button"
                    onClick={() => setSidebarOpen(false)}
                    className="-m-2.5 p-2.5"
                  >
                    <span className="sr-only">Close sidebar</span>
                    <XMarkIcon
                      aria-hidden="true"
                      className="h-6 w-6 text-green-700"
                    />
                  </button>
                </div>
              </TransitionChild>

              <div className="flex grow flex-col gap-y-5 overflow-y-auto px-6 pb-4 ring-1 ring-white/10">
                <div className="flex h-16 shrink-0 items-center">
                  <Link href="/">
                    <Image
                      alt="Your Company"
                      src="/logo.svg"
                      width={200}
                      height={200}
                      className="h-12 rounded-full w-auto"
                    />
                  </Link>
                </div>
                <nav className="flex mt-8 flex-1 flex-col">
                  <ul role="list" className="flex flex-1 flex-col gap-y-7">
                    <li>
                      <ul role="list" className="-mx-2 space-y-1">
                        {navigation.map((item) => (
                          <li
                            key={item.id}
                            className={classNames(
                              item.current
                                ? "bg-green-900 text-white"
                                : "text-gray-400 hover:bg-green-900 hover:text-white",
                              "relative flex items-center group rounded-md transition-colors duration-300"
                            )}
                          >
                            <Link
                              href={
                                item.id === "new"
                                  ? "/chat/"
                                  : `/chat/${item.id}`
                              }
                              className="flex-1 truncate px-4 py-2 text-sm font-semibold leading-6 rounded-md opacity-100"
                            >
                              {item.name}
                            </Link>
                            {item.id === "new" && (
                              <Link href="/chat">
                                <item.icon className=" mr-2 w-6 h-6" />
                              </Link>
                            )}

                            {item.id != "new" && (
                              <Menu as="div" className="relative ml-2">
                                <Menu.Button
                                  onClick={() => handleEllipsisClick(item.id)}
                                  className="p-2 text-gray-500 hover:text-white transition-colors rounded-md duration-300"
                                >
                                  <EllipsisVerticalIcon
                                    aria-hidden="true"
                                    className="h-5 w-5"
                                  />
                                </Menu.Button>

                                <MenuItems
                                  transition
                                  className="absolute right-0 z-10 -mt-2 origin-top-right divide-y divide-gray-100 rounded-md hover:bg-red-900 bg-red-800 border border-red-500 shadow-lg ring-1 ring-black ring-opacity-5 transition focus:outline-none data-[closed]:scale-95 data-[closed]:transform data-[closed]:opacity-0 data-[enter]:duration-100 data-[leave]:duration-75 data-[enter]:ease-out data-[leave]:ease-in opacity-100"
                                >
                                  <MenuItem>
                                    <button
                                      onClick={() => deleteChat(item.id)}
                                      className="block w-14 rounded-md px-2 py-1 text-left text-xs text-white "
                                    >
                                      Delete
                                    </button>
                                  </MenuItem>
                                </MenuItems>
                              </Menu>
                            )}
                          </li>
                        ))}
                      </ul>
                    </li>
                  </ul>
                </nav>
              </div>
            </DialogPanel>
          </div>
        </Dialog>

        <div className="hidden lg:fixed lg:inset-y-0 lg:flex lg:w-64">
          <div className="flex grow flex-col gap-y-5 overflow-y-auto px-6 pb-4 bg-opacity-100">
            <div className="flex h-16 shrink-0 items-center">
              {/* <Image
                alt="Your Company"
                src="/logo.jpg"
                width={200}
                height={200}
                className="mt-4 h-12 rounded-full w-auto"
              /> */}
            </div>
            <nav className="flex mt-8 bg-transparent flex-1 flex-col">
              <ul role="list" className="flex flex-1 flex-col gap-y-7">
                <li>
                  <ul role="list" className="-mx-2 space-y-1">
                    {navigation.map((item) => (
                      <li
                        key={item.id}
                        className={classNames(
                          item.current
                            ? "bg-green-900 text-white"
                            : "text-gray-400 hover:bg-green-900 hover:text-white",
                          "relative flex items-center group rounded-md transition-colors duration-300"
                        )}
                      >
                        <Link
                          href={
                            item.id === "new" ? "/chat/" : `/chat/${item.id}`
                          }
                          className="flex-1 truncate px-4 py-2 text-sm font-semibold leading-6 rounded-md opacity-100"
                        >
                          {item.name}
                        </Link>
                        {item.id === "new" && (
                          <Link href="/chat">
                            <item.icon className=" mr-2 w-6 h-6" />
                          </Link>
                        )}

                        {item.id != "new" && (
                          <Menu as="div" className="relative ml-2">
                            <Menu.Button
                              onClick={() => handleEllipsisClick(item.id)}
                              className="p-2 text-gray-500 hover:text-white transition-colors rounded-md duration-300"
                            >
                              <EllipsisVerticalIcon
                                aria-hidden="true"
                                className="h-5 w-5"
                              />
                            </Menu.Button>

                            <MenuItems
                              transition
                              className="absolute right-0 z-10 -mt-2 origin-top-right divide-y divide-gray-100 rounded-md hover:bg-red-900 bg-red-800 border border-red-500 shadow-lg ring-1 ring-black ring-opacity-5 transition focus:outline-none data-[closed]:scale-95 data-[closed]:transform data-[closed]:opacity-0 data-[enter]:duration-100 data-[leave]:duration-75 data-[enter]:ease-out data-[leave]:ease-in opacity-100"
                            >
                              <MenuItem>
                                <button
                                  onClick={() => deleteChat(item.id)}
                                  className="block w-14 rounded-md px-2 py-1 text-left text-xs text-white "
                                >
                                  Delete
                                </button>
                              </MenuItem>
                            </MenuItems>
                          </Menu>
                        )}
                      </li>
                    ))}
                  </ul>
                </li>
              </ul>
            </nav>
          </div>
        </div>

        <div className="lg:pl-72 bg-white opacity-80">
          <div className="sticky w-fit top-0 flex h-16 z-50 shrink-0 items-center gap-x-4 px-4  sm:gap-x-6 sm:px-6 lg:px-8">
            <button
              type="button"
              onClick={() => setSidebarOpen(true)}
              className="-m-2.5 p-2.5 text-green-700 lg:hidden flex items-center"
            >
              <span className="sr-only">Open sidebar</span>
              <Bars3Icon aria-hidden="true" className="h-6 w-6" />
              <span className="ml-2">View Chats</span>
            </button>
          </div>
        </div>
      </div>
    </>
  );
}
