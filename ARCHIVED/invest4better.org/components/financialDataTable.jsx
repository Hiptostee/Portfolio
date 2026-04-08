import { calculateIndex } from "../utils/calculations";

export default function FinancialDataTable({
  selectedCompanies,
  financialData,
  setInfoOpen,
}) {
  const metrics = [
    {
      label: "Company",
      value: (company) =>
        financialData[company.ticker]?.companyName || company.name,
    },
    {
      label: "Net Profit Margin (%)",
      value: (company) =>
        financialData[company.ticker]?.netProfitMargin?.toFixed(2) ?? "-",
    },
    {
      label: "P/E Ratio",
      value: (company) =>
        financialData[company.ticker]?.peRatio?.toFixed(2) ?? "-",
    },
    {
      label: "Dividend Yield (%)",
      value: (company) =>
        financialData[company.ticker]?.dividendYield?.toFixed(2) ?? "-",
    },
    {
      label: "Current Price ($)",
      value: (company) =>
        financialData[company.ticker]?.currentPrice?.toFixed(2) ?? "-",
    },
    {
      label: (
        <span
          onClick={() => setInfoOpen(true)}
          className="cursor-pointer text-green-500 hover:underline"
        >
          I4B INDEX <sup className="text-purple-600">ⓘ</sup>
        </span>
      ),
      value: (company) => {
        const data = financialData[company.ticker];
        if (!data) return "-";
        const ratiosForIndex = {
          [company.ticker]: {
            "Net Profit Margin (in %)": data.netProfitMargin,
          },
        };
        return calculateIndex(company, ratiosForIndex).toFixed(2);
      },
    },
  ];

  return (
    <div className="w-full overflow-auto text-gray-300">
      <h2 className="mb-2 text-xl font-bold text-green-600">
        Company Financial Data
      </h2>
      <div className="overflow-hidden rounded-lg border border-gray-700 shadow-md">
        <table className="min-w-full divide-y divide-gray-700">
          <thead className="bg-gray-800">
            <tr>
              <th
                scope="col"
                className="px-4 py-3 text-left text-xs font-medium uppercase tracking-wider text-green-500"
              >
                Metric
              </th>
              {selectedCompanies.map((company) => (
                <th
                  key={company.ticker}
                  scope="col"
                  className="px-4 py-3 text-left text-xs font-medium uppercase tracking-wider text-green-500"
                >
                  {company.ticker}
                </th>
              ))}
            </tr>
          </thead>
          <tbody className="divide-y divide-gray-700 bg-gray-900">
            {metrics.map(({ label, value }, index) => (
              <tr key={index} className="hover:bg-gray-800">
                <td className="whitespace-nowrap px-4 py-3 text-sm font-medium text-white">
                  {label}
                </td>
                {selectedCompanies.map((company) => (
                  <td
                    key={company.ticker}
                    className="whitespace-wrap px-4 py-3 text-sm text-gray-300"
                  >
                    {value(company)}
                  </td>
                ))}
              </tr>
            ))}
          </tbody>
        </table>
      </div>
      <div className="mt-2 text-sm text-gray-500">
        <p>I4B Index Ratings</p>
        <p>{"<"} 1: Poor | 1-2: Average | 2-3: Good | 3+: Excellent</p>
      </div>
    </div>
  );
}
