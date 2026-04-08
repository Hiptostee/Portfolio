"use client";
import { BlockMath } from "react-katex";
import "katex/dist/katex.min.css";

import {
  Dialog,
  DialogBackdrop,
  DialogPanel,
  DialogTitle,
} from "@headlessui/react";

export default function Info({ open, setOpen }) {
  return (
    <Dialog open={open} onClose={setOpen} className="relative z-10">
      <DialogBackdrop
        transition
        className="fixed inset-0 bg-gray-900 bg-opacity-75 transition-opacity data-[closed]:opacity-0 data-[enter]:duration-300 data-[leave]:duration-200 data-[enter]:ease-out data-[leave]:ease-in"
      />

      <div className="fixed inset-0 z-10 w-screen overflow-y-auto">
        <div className="flex min-h-full items-end justify-center p-4 text-center sm:items-center sm:p-0">
          <DialogPanel
            transition
            className="relative transform overflow-hidden rounded-lg bg-gray-800 px-4 pb-4 pt-5 text-left shadow-xl transition-all data-[closed]:translate-y-4 data-[closed]:opacity-0 data-[enter]:duration-300 data-[leave]:duration-200 data-[enter]:ease-out data-[leave]:ease-in sm:my-8 sm:w-full sm:max-w-sm sm:p-6 data-[closed]:sm:translate-y-0 data-[closed]:sm:scale-95"
          >
            <div>
              <div className="mt-3 sm:mt-5">
                <DialogTitle
                  as="h3"
                  className="text-center mb-4 text-xl font-semibold leading-6 text-green-500"
                >
                  What is the I4B Index?
                </DialogTitle>
                <div className="text-gray-300 ">
                  <p>
                    The <strong>I4B Index</strong> is our proprietary metric for
                    evaluating both sustainability and profitability. A higher
                    I4B score indicates a better investment for both your wallet
                    and the future.
                  </p>
                  <br />
                  <p>
                    This score is calculated by dividing the{" "}
                    <strong>net profit margin</strong> by the company's{" "}
                    <strong>
                      total ESG (Environmental, Social, and Governance) score
                    </strong>
                    , providing a balanced view of financial performance and
                    social responsibility:
                  </p>

                  <BlockMath
                    math={
                      "\\text{I4B Index} = \\frac{\\text{Net Profit Margin}}{\\text{Total ESG Score}}"
                    }
                  />

                  <p>
                    By integrating profitability with sustainability, the I4B
                    Index allows for a more informed investment decision that
                    takes into account both economic returns and
                    environmental/social impacts.
                  </p>
                </div>
              </div>
            </div>
            <div className="mt-5 sm:mt-6">
              <button
                type="button"
                onClick={() => setOpen(false)}
                className="inline-flex w-full justify-center rounded-md bg-green-700 px-3 py-2 text-sm font-semibold text-white shadow-sm hover:bg-green-600 focus-visible:outline focus-visible:outline-2 focus-visible:outline-offset-2 focus-visible:outline-indigo-600"
              >
                Exit
              </button>
            </div>
          </DialogPanel>
        </div>
      </div>
    </Dialog>
  );
}
