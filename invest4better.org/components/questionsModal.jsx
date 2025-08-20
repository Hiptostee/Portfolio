"use client";

import { useState } from "react";
import { Dialog, DialogPanel, DialogTitle } from "@headlessui/react";
import { XMarkIcon } from "@heroicons/react/24/outline";

export default function Example({
  questions,
  open,
  setOpen,
  selected,
  setSelected,
}) {
  return (
    <Dialog open={open} onClose={setOpen} className="relative z-50">
      <div className="fixed inset-0" />

      <div className="fixed inset-0 overflow-hidden">
        <div className="absolute inset-0 overflow-hidden">
          <div className="pointer-events-none fixed inset-y-0 right-0 flex max-w-full pl-10">
            <DialogPanel
              transition
              className="pointer-events-auto w-screen max-w-md transform transition duration-500 ease-in-out data-[closed]:translate-x-full sm:duration-700"
            >
              <div className="flex h-full flex-col overflow-y-scroll bg-[#072012] py-6 shadow-xl">
                <div className="px-4 sm:px-6">
                  <div className="flex items-start justify-between">
                    <DialogTitle className="text-base  font-semibold leading-6 text-gray-50">
                      Questions
                    </DialogTitle>
                    <div className="ml-3 flex h-7 items-center">
                      <button
                        type="button"
                        onClick={() => setOpen(false)}
                        className="relative rounded-md bg-green-900 text-green-500 hover:text-green-700 focus:outline-none focus:ring-2 focus:ring-indigo-500 focus:ring-offset-2"
                      >
                        <span className="absolute -inset-2.5" />
                        <span className="sr-only">Close panel</span>
                        <XMarkIcon aria-hidden="true" className="h-6 w-6" />
                      </button>
                    </div>
                  </div>
                </div>
                <div className="relative mt-6 flex-1 px-4 sm:px-6">
                  <div className="text-center md:text-left">
                    <div className="flex flex-grow flex-col overflow-x-hidden">
                      <div
                        className={` deap-scrollbar max-h-full min-h-0 flex-grow overflow-y-auto overflow-x-hidden rounded-md bg-opacity-40 xl:rounded-lg 2xl:rounded-xl`}
                      >
                        {questions &&
                          questions.map((questionData, index) => (
                            <div key={index} className="mx-4 min-w-0">
                              <div className="pt-8 text-center font-base text-xl font-bold">
                                {questionData.question}
                              </div>
                              <hr className="mt-4 mb-4 border-green-900" />
                              <ol className="ml-4 mt-4 list-outside list-[upper-alpha]">
                                {questionData.answers.map((answer, index2) => (
                                  <li key={index2} className="mb-2">
                                    <button
                                      type="button"
                                      className={`w-full rounded-lg px-2 py-1 text-left hover:cursor-pointer xl:rounded-lg 2xl:rounded-xl ${
                                        index2 === selected[index]
                                          ? index2 === questionData.answerIndex
                                            ? "bg-green-500 text-white" // Selected correct answer
                                            : "bg-red-500 text-white" // Selected incorrect answer
                                          : "hover:bg-green-900" // Hover effect for non-selected
                                      }`}
                                      onClick={() =>
                                        setSelected((prevSelected) => {
                                          const newSelected = [...prevSelected]; // Create a new array
                                          newSelected[index] = index2; // Update the selected index
                                          return newSelected; // Return the new array
                                        })
                                      }
                                    >
                                      {answer}
                                    </button>
                                  </li>
                                ))}
                                <hr className="mt-4 mb-4 border-green-900" />
                              </ol>
                            </div>
                          ))}
                      </div>
                    </div>
                  </div>
                </div>
              </div>
            </DialogPanel>
          </div>
        </div>
      </div>
    </Dialog>
  );
}
