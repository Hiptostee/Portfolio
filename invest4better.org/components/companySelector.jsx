import { CheckIcon } from "@heroicons/react/24/outline";

export default function CompanySelector({
  techSectorData,
  selectedCompanies,
  onCompanyClick,
}) {
  return (
    <div className="w-full md:w-1/2">
      <div className="grid grid-cols-2 gap-4 lg:grid-cols-3">
        {techSectorData.map((company) => {
          const isSelected = selectedCompanies.some(
            (c) => c.ticker === company.ticker
          );
          return (
            <button
              key={company.ticker}
              className={`relative cursor-pointer rounded-lg border p-4 text-gray-400 shadow-md transition duration-200 ease-in-out hover:bg-gray-700 ${
                isSelected ? "bg-gray-700 ring-2 ring-green-500" : "bg-gray-800"
              }`}
              onClick={() => onCompanyClick(company)}
              aria-pressed={isSelected}
            >
              {isSelected && (
                <CheckIcon className="absolute right-2 top-2 h-5 w-5 text-green-500" />
              )}
              <h2 className="font-semibold text-white">
                {company.name} ({company.ticker})
              </h2>
              <p>ESG Score: {company.esg ?? "Not Found"}</p>
            </button>
          );
        })}
      </div>
    </div>
  );
}
