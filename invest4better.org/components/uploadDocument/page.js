/* eslint-disable react/no-unescaped-entities */
"use client";

import React, { useState, useEffect, useRef } from "react";
import TopBar from "../topbar";
import { ClipboardIcon } from "@heroicons/react/24/outline";
import { Menu, MenuButton, MenuItem, MenuItems } from "@headlessui/react";
import { ChevronDownIcon } from "@heroicons/react/20/solid";
import Footer from "../footer";
import Link from "next/link";

export default function ChatComponent({
  onFileData,
  getSummary,
  setFullText,
  setTicker,
  type,
}) {
  const [start, setStart] = useState();
  const [end, setEnd] = useState();
  const [extractedText, setExtractedText] = useState("");
  const [pdfData, setPdfData] = useState(null);
  const [firstPageText, setFirstPageText] = useState("");
  const [input, setInput] = useState(null);

  const [botType, setBotType] = useState(type ? type : "Research Bot");

  const [loading, setLoading] = useState(false);
  const pdfUrlRef = useRef(null);

  useEffect(() => {
    const script = document.createElement("script");
    script.src = "https://unpkg.com/pdfjs-dist@3.11.174/build/pdf.min.js";
    script.onload = () => {};
    document.head.appendChild(script);

    return () => {
      document.head.removeChild(script);
    };
  }, []);

  async function generateText(fullText) {
    try {
      setLoading(true);

      // Simulate splitting fullText into pages (assuming text is split by '\n\n' or some other page delimiter)
      const pages = fullText.split("\n\n"); // Adjust the delimiter as needed to fit your text structure

      // Combine the first three pages into one text block for firstPageText
      const firstPageText = pages.slice(0, 3).join("\n\n"); // Joins the first three pages

      let bot;
      if (botType === "Research Bot") {
        bot = "researchSummary";
      } else if (botType === "10-K Reader") {
        bot = "financeSummary";
      } else if (botType === "Congress Bill Bot") {
        bot = "congressSummary";
      }

      const response = await fetch(`/api/financeSummary`, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({ fullText, firstPageText }),
      });

      if (!response.ok) {
        throw new Error("Failed to generate summary");
      }

      const data = await response.json();
      getSummary(data.summary);
      setTicker(data.tickerInfo);
    } catch (error) {
      console.error("Error generating text:", error);
    } finally {
      setLoading(false);
    }
  }

  function convert() {
    if (!pdfData) return;

    const pdff = new Pdf2TextClass(parseInt(start), parseInt(end));
    pdff.pdfToText(pdfData, null, (text) => {
      setExtractedText(text);
    });
  }

  function Pdf2TextClass(startPage, endPage) {
    var self = this;
    this.complete = 0;

    this.pdfToText = function (data, callbackPageDone, callbackAllDone) {
      if (!window.pdfjsLib) {
        console.error("pdfjsLib is not loaded.");
        return;
      }
      var loadingTask = window.pdfjsLib.getDocument({
        data: atob(data.split(",")[1]),
      });

      loadingTask.promise.then(function (pdf) {
        var total = Math.min(pdf.numPages, endPage);
        total = Math.max(total, startPage);
        setEnd(total);
        var layers = {};
        var firstPageText = "";

        function processPage(i) {
          pdf.getPage(i).then(function (page) {
            var n = page.pageNumber;
            page.getTextContent().then(function (textContent) {
              if (textContent.items) {
                var page_text = "";
                var last_block = null;
                for (var k = 0; k < textContent.items.length; k++) {
                  var block = textContent.items[k];
                  if (
                    last_block &&
                    last_block.str[last_block.str.length - 1] != " "
                  ) {
                    if (block.transform[4] < last_block.transform[4])
                      page_text += "\n";
                    else if (
                      last_block.transform[5] != block.transform[5] &&
                      !last_block.str.match(/^(\s?[a-zA-Z])$|^(.+\s[a-zA-Z])$/)
                    )
                      page_text += " ";
                  }
                  page_text += block.str;
                  last_block = block;
                }

                if (i === startPage) {
                  firstPageText = page_text;
                  setFirstPageText(firstPageText);
                }

                layers[n] = page_text + "\n\n";
              }
              ++self.complete;
              if (self.complete == total - startPage + 1) {
                setTimeout(function () {
                  var full_text = "";
                  for (var j = startPage; j <= total; j++) {
                    if (layers[j]) {
                      full_text += layers[j];
                    }
                  }
                  setExtractedText(full_text);
                  setFullText(full_text);
                  generateText(full_text, firstPageText);
                  callbackAllDone(full_text);
                }, 1000);
              }
            });
          });
        }

        for (let i = startPage; i <= total; i++) {
          processPage(i);
        }
      });
    };
  }

  useEffect(() => {
    if (input) {
      pdfUrlRef.current = URL.createObjectURL(input);
      onFileData(input); // Pass the file data to the parent
      return () => {
        URL.revokeObjectURL(pdfUrlRef.current);
      };
    }
  }, [input, onFileData]);

  return (
    <>
      <div className="bg-white relative h-screen">
        <div className="flex w-full h-full">
          <div className={`${input ? "w-full" : "w-full"} h-full px-16 pt-14`}>
            <div className="mx-auto max-w-5xl flex flex-col h-full">
              <div className="text-3xl text-green-700 mt-8  mx-auto mb-5">
                <strong>Invest4Better</strong> - Upload Annual Report to
                Summarize
              </div>
              <div className="relative isolate">
                {/* <div className="flex w-full mb-8">
                  {" "}
                  <Menu as="div" className="relative inline-block text-left">
                    <div>
                      <MenuButton className="inline-flex w-full justify-center gap-x-1.5 rounded-md bg-gray-800 px-3 py-2 text-sm font-semibold text-gray-50 shadow-sm ring-1 ring-inset ring-gray-600 hover:bg-gray-70">
                        {botType}
                        <ChevronDownIcon
                          aria-hidden="true"
                          className="-mr-1 h-5 w-5 text-gray-400"
                        />
                      </MenuButton>
                    </div>

                    <MenuItems
                      transition
                      className="absolute left-0 z-10 mt-2 w-56 origin-top-right divide-y divide-gray-100 rounded-md bg-gray-800 shadow-lg ring-1 ring-black ring-opacity-5 transition focus:outline-none data-[closed]:scale-95 data-[closed]:transform data-[closed]:opacity-0 data-[enter]:duration-100 data-[leave]:duration-75 data-[enter]:ease-out data-[leave]:ease-in"
                    >
                      <div className="py-1">
                        <MenuItem>
                          <div
                            onClick={() => setBotType("10-K Reader")}
                            className="block px-4 py-2 text-sm text-gray-300 data-[focus]:bg-gray-700"
                          >
                            10-K Reader
                          </div>
                        </MenuItem>
                        <MenuItem>
                          <div
                            onClick={() => setBotType("Research Bot")}
                            className="block px-4 py-2 text-sm text-gray-300 data-[focus]:bg-gray-700"
                          >
                            Research Bot
                          </div>
                        </MenuItem>
                        <MenuItem>
                          <div
                            onClick={() => setBotType("Congress Bill Bot")}
                            className="block px-4 py-2 text-sm text-gray-300 data-[focus]:bg-gray-700"
                          >
                            Congress Bill Bot
                          </div>
                        </MenuItem>
                      </div>
                    </MenuItems>
                  </Menu>
                </div> */}
                {botType === "Congress Bill Bot" && (
                  <>
                    <div className="font-bold text-sm mb-1">
                      Want help finding bills to summarize?
                    </div>
                    <div className="font-base text-sm mb-8">
                      Click here to visit{" "}
                      <Link
                        className="text-sm text-indigo-500 hover:text-indigo-600 hover:underline"
                        href="https://www.progressincongress.org/bills"
                        target="_blank"
                      >
                        Progress in Congress!
                      </Link>
                    </div>
                  </>
                )}

                {/* Removed drag-and-drop feature, kept click to upload only */}
                <div className="flex  items-center justify-center w-full">
                  <label
                    htmlFor="dropzone-file"
                    className="flex flex-col items-center justify-center w-full h-64 border-2  border-dashed rounded-lg cursor-pointer   bg-none  border-green-700 hover:border-green-500 "
                  >
                    <div className="flex z-20 flex-col items-center justify-center pt-5 pb-6">
                      <svg
                        className="w-8 h-8 mb-4 text-green-900"
                        aria-hidden="true"
                        xmlns="http://www.w3.org/2000/svg"
                        fill="none"
                        viewBox="0 0 20 16"
                      >
                        <path
                          stroke="currentColor"
                          strokeLinecap="round"
                          strokeLinejoin="round"
                          strokeWidth="2"
                          d="M13 13h3a3 3 0 0 0 0-6h-.025A5.56 5.56 0 0 0 16 6.5 5.5 5.5 0 0 0 5.207 5.021C5.137 5.017 5.071 5 5 5a4 4 0 0 0 0 8h2.167M10 15V6m0 0L8 8m2-2 2 2"
                        />
                      </svg>
                      <p className="mb-2 text-sm text-green-900">
                        <span className="font-semibold">Click to upload</span>{" "}
                      </p>
                      <p className="text-xs text-green-900">
                        PDF only (MAX. 10MB)
                      </p>
                      <div className="text-green-700">
                        {" "}
                        {input && input.name ? input.name : ""}
                      </div>
                    </div>
                    <input
                      id="dropzone-file"
                      accept=".pdf"
                      onChange={(e) => {
                        const file = e.target.files?.item(0);
                        if (file) {
                          const fr = new FileReader();
                          fr.onload = () => setPdfData(fr.result);
                          fr.readAsDataURL(file);
                        }
                        setInput(file);
                      }}
                      type="file"
                      className="hidden"
                    />
                  </label>
                </div>
                <div className="grid grid-cols-2 sm:grid-cols-3 gap-6 mt-8 mb-4">
                  <div className="flex flex-col items-start">
                    <label className="text-green-700 font-bold text-base sm:text-lg mb-2">
                      Start Page
                    </label>
                    <input
                      type="number"
                      min={1}
                      onChange={(e) => setStart(e.target.value)}
                      className="w-full sm:w-24 h-10 px-3 text-black text-sm border border-green-700 rounded-lg"
                    />
                  </div>

                  <div className="flex flex-col items-start">
                    <label className="text-green-700 font-bold text-base sm:text-lg mb-2">
                      End Page
                    </label>
                    <input
                      type="number"
                      min={1}
                      onChange={(e) => setEnd(e.target.value)}
                      className="w-full sm:w-24 h-10 px-3 text-black text-sm border border-green-700 rounded-lg"
                    />
                  </div>

                  <div className="flex items-start justify-center sm:justify-start">
                    {!loading ? (
                      <button
                        onClick={convert}
                        className="w-full sm:w-auto bg-green-600 hover:bg-green-500 text-white font-bold h-10 px-8 mt-6 sm:px-12 rounded-md"
                      >
                        Summarize
                      </button>
                    ) : (
                      <div
                        className=" items-center justify-center mt-8"
                        role="status"
                      >
                        <svg
                          aria-hidden="true"
                          className="w-8 h-8 animate-spin text-gray-600 fill-green-600"
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
                    )}
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
                    className="relative left-[calc(70%+3rem)] aspect-[1155/678] w-[36.125rem] -translate-x-1/3 bg-gradient-to-tr from-green-800 to-green-700 opacity-30 sm:left-[calc(10%+36rem)] sm:w-[72.1875rem]"
                  />
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </>
  );
}
