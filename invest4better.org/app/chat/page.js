"use client"; // Add this at the very top of your file

import React, { useState, useEffect } from "react";
import TopBar from "../../components/topbar";
import ChatComponent from "../../components/uploadDocument/page";
import SideBar from "../../components/sidebar";
import { useSearchParams } from "next/navigation";
import Chat from "../../components/chatBot/page";
import { Dialog, DialogPanel } from "@headlessui/react";
import { Bars3Icon, XMarkIcon } from "@heroicons/react/24/outline";
import Image from "next/image";
import Link from "next/link";
import {
  WrenchScrewdriverIcon,
  ClipboardIcon,
} from "@heroicons/react/24/outline";

const navigation = [
  { name: "Home", href: "/" },
  // { name: "Tools", href: "/tools" },
  // { name: "Bots", href: "/chat" },
  // { name: "About", href: "/about" },
];

export default function ChatPage() {
  const [mobileMenuOpen, setMobileMenuOpen] = useState(false);
  const [fileData, setFileData] = useState(null);
  const [showPdf, setShowPdf] = useState(true);
  const [summary, setSummary] = useState("");
  const [fullText, setFullText] = useState("");
  const [loading, setLoading] = useState(false);
  const [chats, setChats] = useState([]);
  const searchParams = useSearchParams();
  const [ticker, setTicker] = useState("");
  var type = "";
  const search = searchParams.get("type");

  if (search === "10-KReader") {
    type = "10-K Reader";
  }
  if (search === "researchBot") {
    type = "Research Bot";
  }

  const handleFileData = (data) => {
    setFileData(data);
  };

  useEffect(() => {
    const handleResize = () => {
      if (window.innerWidth < 768) {
        setShowPdf(false);
      } else {
        setShowPdf(true);
      }
    };

    window.addEventListener("resize", handleResize);
    handleResize(); // Call handler right away so state gets updated with initial window size

    return () => window.removeEventListener("resize", handleResize);
  }, []);

  useEffect(() => {
    async function fetchChats() {
      setLoading(true);
      setChats([
        {
          name: "New Chat",
          icon: WrenchScrewdriverIcon,
          id: "new",
          current: true,
        },
      ]);
      setLoading(false);
    }

    fetchChats();
  }, []);

  return (
    <>
      <header className="absolute inset-x-0 top-0 z-10">
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
              className="-m-2.5 inline-flex items-center justify-center rounded-md p-2.5 text-green-700"
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
                className="text-sm font-semibold leading-6 text-green-700 hover:underline  rounded-lg px-3 py-2"
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
          <DialogPanel className="fixed inset-y-0 right-0 z-10 w-full overflow-y-auto bg-white px-6 py-6 sm:max-w-sm sm:ring-1 sm:ring-gray-900/10">
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
      <div className="h-full flex flex-col relative text-white">
        <div className="flex bg-white flex-1 relative isolate flex-col md:flex-row text-white">
          {loading ? (
            <div
              className="absolute lg:block hidden text-center my-auto mt-36 ml-24 text-white"
              role="status"
            >
              <svg
                aria-hidden="true"
                className="w-8 h-8  animate-spin text-gray-600 fill-green-600"
                viewBox="0 0 100 101"
                fill="none"
                xmlns="http://www.w3.org/2000/svg"
              >
                <path
                  d="M100 50.5908C100 78.2051 77.6142 100.591 50 100.591C22.3858 100.591 0 78.2051 0 50.5908C0 22.9766 22.3858 0.59082 50 0.59082C77.6142 0.59082 100 22.9766 100 50.5908ZM9.08144 50.5908C9.08144 73.1895 27.4013 91.5094 50 91.5094C72.5987 91.5094 90.9186 73.1895 90.9186 50.5908C90.9186 27.9921 72.5987 9.67226 50 9.67226C27.4013 9.67226 9.08144 27.9921 9.08144 50.5908Z"
                  fill="currentColor"
                />
                <path
                  d="M93.9676 39.0409C96.393 38.4038 97.8624 35.9116 97.0079 33.5539C95.2932 28.8227 92.871 24.3692 89.8167 20.348C85.8452 15.1192 80.8826 10.7238 75.2124 7.41289C69.5422 4.10194 63.2754 1.94025 56.7698 1.05124C51.7666 0.367541 46.6976 0.446843 41.7345 1.27873C39.2613 1.69328 37.813 4.19778 38.4501 6.62326C39.0873 9.04874 41.5694 10.4717 44.0505 10.1071C47.8511 9.54855 51.7191 9.52689 55.5402 10.0491C60.8642 10.7766 65.9928 12.5457 70.6331 15.2552C75.2735 17.9648 79.3347 21.5619 82.5849 25.841C84.9175 28.9121 86.7997 32.2913 88.1811 35.8758C89.083 38.2158 91.5421 39.6781 93.9676 39.0409Z"
                  fill="currentFill"
                />
              </svg>
            </div>
          ) : null}
          {/* <div className=" text-white">
            {!fileData || !showPdf ? <SideBar navigation={chats} /> : null}
          </div> */}
          <div
            className={`w-full ${
              showPdf && fileData ? "w-1/2" : "w-full"
            } text-white`}
          >
            <div className="flex flex-col overflow-x-hidden md:flex-row h-full text-white">
              {showPdf && fileData && (
                <div className="relative w-full mt-24 md:w-1/2 bg-gray-200 hidden md:block text-white">
                  <embed
                    src={URL.createObjectURL(fileData)}
                    className="w-full h-full"
                    type="application/pdf"
                    width="100%"
                    height="100%"
                  />
                </div>
              )}
              {fileData && (
                <button
                  onClick={() => {
                    setShowPdf(!showPdf);
                  }}
                  className="absolute top-36 md:block hidden right-4 md:right-8 z-10 bg-green-700 border border-green-600 text-white py-1 text-xs px-2 rounded-full"
                >
                  {showPdf ? "Hide PDF" : "Show PDF"}
                </button>
              )}
              <div
                className={`w-full ${
                  showPdf && fileData ? "lg:w-1/2" : "l w-full"
                } text-white`}
              >
                {!summary ? (
                  <ChatComponent
                    onFileData={handleFileData}
                    getSummary={setSummary}
                    setFullText={setFullText}
                    setTicker={setTicker}
                    type={type}
                  />
                ) : (
                  <Chat summary={summary} ticker={ticker} fullText={fullText} />
                )}
              </div>
            </div>
          </div>
        </div>
      </div>
    </>
  );
}
