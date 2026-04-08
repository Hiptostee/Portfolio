import { NextResponse } from "next/server";

export async function GET(request) {
  const { searchParams } = new URL(request.url);
  const ticker = searchParams.get("ticker");

  if (!ticker || typeof ticker !== "string") {
    return NextResponse.json(
      { error: "A valid stock ticker is required." },
      { status: 400 }
    );
  }

  // **SECURELY LOAD MULTIPLE KEYS FROM ENVIRONMENT VARIABLE**
  const apiKeysString = process.env.API_KEYS;
  if (!apiKeysString) {
    return NextResponse.json(
      { error: "Server configuration error: API keys are not configured." },
      { status: 500 }
    );
  }

  // Parse the JSON string into a JavaScript array
  let apiKeys;
  try {
    apiKeys = JSON.parse(apiKeysString);
  } catch (err) {
    console.error("Failed to parse API keys:", err);
    return NextResponse.json(
      { error: "Server configuration error: Invalid API key format." },
      { status: 500 }
    );
  }

  let combinedData = null;
  let success = false;
  let attempts = 0;

  // **LOOP THROUGH API KEYS ON THE SERVER**
  while (attempts < apiKeys.length && !success) {
    const apiKey = apiKeys[attempts].api_key;

    try {
      const endpoints = [
        `https://financialmodelingprep.com/api/v3/profile/${ticker}?apikey=${apiKey}`,
        `https://financialmodelingprep.com/api/v3/balance-sheet-statement/${ticker}?period=annual&apikey=${apiKey}`,
        `https://financialmodelingprep.com/api/v3/income-statement/${ticker}?period=annual&apikey=${apiKey}`,
        `https://financialmodelingprep.com/api/v3/ratios/${ticker}?apikey=${apiKey}`,
        `https://financialmodelingprep.com/api/v3/quote/${ticker}?apikey=${apiKey}`,
        `https://financialmodelingprep.com/api/v3/historical-price-full/${ticker}?timeseries=365&apikey=${apiKey}`,
      ];

      const responses = await Promise.all(endpoints.map((url) => fetch(url)));

      // Check if any response failed (e.g., due to rate limit or invalid key)
      const allResponsesOk = responses.every((response) => response.ok);
      if (!allResponsesOk) {
        throw new Error(
          "One or more API requests failed. Retrying with a new key..."
        );
      }

      const data = await Promise.all(
        responses.map((response) => response.json())
      );

      const [
        profileData,
        balanceSheetData,
        incomeStatementData,
        ratiosData,
        stockPriceData,
        historicalPricesData,
      ] = data;

      if (
        !profileData[0] ||
        !stockPriceData[0] ||
        !historicalPricesData.historical
      ) {
        throw new Error("No data found for the ticker. Retrying...");
      }

      combinedData = {
        profile: profileData[0],
        balanceSheet: balanceSheetData[0],
        incomeStatement: incomeStatementData,
        ratios: ratiosData[0],
        stockPrice: stockPriceData[0],
        historicalPrices: historicalPricesData,
      };

      success = true; // Data successfully fetched, exit loop
    } catch (error) {
      console.error(`Attempt ${attempts + 1} failed:`, error.message);
      attempts++;
    }
  }

  if (success) {
    return NextResponse.json(combinedData, { status: 200 });
  } else {
    return NextResponse.json(
      { error: "All API keys failed. Please try again later." },
      { status: 503 }
    );
  }
}
