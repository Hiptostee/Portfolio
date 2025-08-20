"use client";

import {
  Dialog,
  DialogBackdrop,
  DialogPanel,
  DialogTitle,
} from "@headlessui/react";
import { XMarkIcon } from "@heroicons/react/24/outline";
import { Doughnut } from "react-chartjs-2";
import { Chart as ChartJS, ArcElement, Tooltip, Legend } from "chart.js";

// Register Chart.js components
ChartJS.register(ArcElement, Tooltip, Legend);

export default function EnvDrawer({
  open,
  setOpen,
  descriptions, // Use 'descriptions' variable
  totalesg,
  eesg,
  sesg,
  gesg,
}) {
  // Chart data with updated colors
  const data = {
    labels: ["Environmental", "Social", "Governance"],
    datasets: [
      {
        label: "ESG Scores",
        data: [eesg, sesg, gesg],
        backgroundColor: [
          "rgba(0, 128, 0, 0.6)", // Green for Environmental
          "rgba(255, 255, 0, 0.6)", // Yellow for Social
          "rgba(128, 0, 128, 0.6)", // Purple for Governance
        ],
        borderWidth: 1,
      },
    ],
  };

  const options = {
    responsive: true,
    plugins: {
      legend: {
        display: true,
        position: "top",
      },
      tooltip: {
        callbacks: {
          label: (tooltipItem) => {
            return `${tooltipItem.label}: ${tooltipItem.raw}`;
          },
        },
      },
    },
  };

  return (
    <Dialog open={open} onClose={setOpen} className="relative z-10">
      <DialogBackdrop
        transition
        className="fixed inset-0 bg-gray-500 bg-opacity-75 transition-opacity duration-500 ease-in-out data-[closed]:opacity-0"
      />

      <div className="fixed inset-0 overflow-hidden">
        <div className="absolute inset-0 overflow-hidden">
          <div className="pointer-events-none fixed inset-y-0 right-0 flex max-w-full pl-10">
            <DialogPanel
              transition
              className="pointer-events-auto w-screen max-w-md transform transition duration-500 ease-in-out data-[closed]:translate-x-full sm:duration-700"
            >
              <div className="flex h-full flex-col overflow-y-scroll bg-white py-6 shadow-xl">
                <div className="px-4 sm:px-6">
                  <div className="flex items-start justify-between">
                    <DialogTitle className="text-base font-semibold leading-6 text-gray-900">
                      ESG Scores Overview
                    </DialogTitle>
                    <div className="ml-3 flex h-7 items-center">
                      <button
                        type="button"
                        onClick={() => setOpen(false)}
                        className="relative rounded-md bg-white text-gray-400 hover:text-gray-500 focus:outline-none focus:ring-2 focus:ring-indigo-500 focus:ring-offset-2"
                      >
                        <span className="absolute -inset-2.5" />
                        <span className="sr-only">Close panel</span>
                        <XMarkIcon aria-hidden="true" className="h-6 w-6" />
                      </button>
                    </div>
                  </div>
                </div>
                <div className="relative mt-6 flex-1 px-4 sm:px-6">
                  <div className="flex flex-col items-center">
                    {/* ESG Scores Explanation */}

                    {/* Donut Chart */}
                    <Doughnut data={data} options={options} />
                    <div className="mt-4">
                      <h3 className="text-lg text-green-700 font-semibold">
                        Total ESG Score: {totalesg}
                      </h3>
                    </div>
                  </div>
                  {/* Descriptions */}
                  <div className="mt-6">
                    {descriptions && (
                      <ul className="pl-5 mt-2 ">
                        {Object.entries(descriptions.explanations).map(
                          ([key, description]) => (
                            <li key={key} className="text-green-700 mt-4">
                              <strong className="text-green-900">
                                {key.charAt(0).toUpperCase() + key.slice(1)}:
                              </strong>{" "}
                              {description}
                            </li>
                          )
                        )}
                      </ul>
                    )}
                  </div>
                  <p className="text-xs mt-16 text-green-900 mb-4 text-left">
                    ESG (Environmental, Social, and Governance) scores provide
                    insights into a company's commitment to sustainability,
                    social responsibility, and effective governance practices.
                    <br />
                  </p>
                  <p className="text-xs text-green-900 mb-4 text-left">
                    <strong>Environmental:</strong> Reflects a company's impact
                    on the environment, including factors like carbon emissions
                    and resource use.
                    <br />
                    <strong>Social:</strong> Evaluates how a company manages
                    relationships with employees, suppliers, customers, and
                    communities.
                    <br />
                    <strong>Governance:</strong> Assesses the company's
                    leadership, audit practices, and shareholder rights.
                  </p>
                </div>
              </div>
            </DialogPanel>
          </div>
        </div>
      </div>
    </Dialog>
  );
}
